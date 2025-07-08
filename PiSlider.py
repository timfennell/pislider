import threading
import time
import tkinter
from tkinter import messagebox
import numpy as np
import RPi.GPIO as GPIO
import os
import math
import logging

# Local project imports
from gui import PiSliderGUI
from holygrail import HolyGrailController
from hardware import HardwareController, CameraController, ROTATION_EN_PIN, ROTATION_DIR_PIN, ROTATION_STEP_PIN, AUX_TRIGGER_PIN

# --- Logging Setup ---
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s')

# --- Constants ---
BELT_PITCH = 2
PULLEY_TEETH = 16
STEPS_PER_REVOLUTION_SLIDER = 200 * 2
STEPS_PER_REVOLUTION_ROTATION = 48000
MOVEMENT_BUFFER = 2
POST_SHOT_PAUSE = 1.0  # A fixed 1-second pause after non-bulb shots for camera to settle

class PiSliderApp:
    def __init__(self, root):
        self.root = root
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.stop_flag = threading.Event()
        self.operation_thread = None
        self.preview_thread = None
        self.camera_check_thread = None

        self.camera_lock = threading.Lock()

        try:
            self.hardware = HardwareController()
            self.hardware.set_stop_flag_reference(self.stop_flag)
            self.camera = CameraController(self.hardware, self.camera_lock)
        except Exception as e:
            logging.critical(f"FATAL: Hardware initialization failed: {e}", exc_info=True)
            messagebox.showerror("Hardware Error", f"Failed to initialize hardware: {e}\nApplication cannot continue.")
            if self.root.winfo_exists(): self.root.destroy()
            return

        self.holygrail_controller = None
        self.gantry_running = False
        self.motors_moving = False
        self.entry_has_focus = False
        self.photos_taken = 0
        self.total_steps = 0
        self.combined_intervals = []
        self.steps_distribution = np.array([])
        self.force_interval = False
        
        self.aux_trigger_fired = threading.Event()
        self.aux_trigger_thread = None

        self.anchor_ev = None
        self.anchor_sun_angle = None

        # --- Tkinter Model Variables ---
        self.direction_var = tkinter.StringVar(value='right')
        self.length_var = tkinter.StringVar(value="2800")
        self.distribution_var = tkinter.StringVar(value="even")
        self.rotation_angle_var = tkinter.DoubleVar(value=90.0)
        self.rotation_direction_var = tkinter.StringVar(value="CW")
        self.rotation_distribution_var = tkinter.StringVar(value="even")
        self.num_photos_var = tkinter.IntVar(value=10)
        self.interval_var = tkinter.DoubleVar(value=8.0)
        
        self.minimum_slider_speed_var = tkinter.IntVar(value=0)
        self.mode_var = tkinter.StringVar(value='timed')
        self.post_timelapse_position_var = tkinter.StringVar(value="Do Nothing")
        self.camera_trigger_mode_var = tkinter.StringVar(value="USB Control")
        self.camera_status_var = tkinter.StringVar(value="Status: Unknown")
        self.photo_count_var = tkinter.StringVar(value="Photos: 0 / 0")
        self.time_remaining_var = tkinter.StringVar(value="Time Rem: --:--")

        self.holygrail_enabled_var = tkinter.BooleanVar(value=False)
        self.holygrail_latitude_var = tkinter.StringVar(value="48.17293")
        self.holygrail_longitude_var = tkinter.StringVar(value="-82.521721")
        self.holygrail_min_aperture_var = tkinter.DoubleVar(value=2.8)
        self.holygrail_max_aperture_var = tkinter.DoubleVar(value=11.0)
        
        # --- START OF MODIFICATION: Add new "Smart ISO" and "Crossfade" variables ---
        self.holygrail_day_iso_var = tkinter.IntVar(value=100)
        self.holygrail_night_iso_var = tkinter.IntVar(value=640)
        self.holygrail_max_transition_iso_var = tkinter.IntVar(value=5000)
        self.holygrail_iso_transition_frames_var = tkinter.IntVar(value=30)
        # --- END OF MODIFICATION ---
        
        self.holygrail_day_interval_var = tkinter.DoubleVar(value=10.0)
        self.holygrail_sunset_interval_var = tkinter.DoubleVar(value=4.0)
        self.holygrail_night_interval_var = tkinter.DoubleVar(value=35.0)
        self.calibration_offset_var = tkinter.StringVar(value="Not Calibrated")

        self.live_iso_var, self.live_shutter_var, self.live_aperture_var, self.live_kelvin_var, self.live_ev_offset_var = (tkinter.StringVar(value="--") for _ in range(5))
        self.live_interval_var = tkinter.StringVar(value="--")
        self.live_slider_steps_var = tkinter.StringVar(value="--")
        self.live_rotation_steps_var = tkinter.StringVar(value="--")

        self.entry_widgets = []
        self.preview_photo_image = None
        self.distribution_photo_image = None
        self.holygrail_entries = {}
        self.distribution_images = { "catenary": "catenary.png", "inverted_catenary": "inverted_catenary.png", "gaussian": "gaussian.png", "inverted_gaussian": "inverted_gaussian.png", "ellipsoidal": "ellipsoidal.png", "inverted_ellipsoidal": "ellipsoidal.png", "parabolic": "parabolic.png", "inverted_parabolic": "inverted_parabolic.png", "cycloid": "cycloid.png", "inverted_cycloid": "inverted_cycloid.png", "lame_curve": "lame_curve.png", "inverted_lame_curve": "inverted_lame_curve.png", "linear": "linear.png", "inverted_linear": self.inverted_linear_distribution, "even": "even.png" }
        self.distribution_functions = { "catenary": self.catenary_distribution, "inverted_catenary": self.inverted_catenary_distribution, "gaussian": self.gaussian_distribution, "inverted_gaussian": self.inverted_gaussian_distribution, "ellipsoidal": self.ellipsoidal_distribution, "inverted_ellipsoidal": self.ellipsoidal_distribution, "parabolic": self.parabolic_distribution, "inverted_parabolic": self.inverted_parabolic_distribution, "cycloid": self.cycloid_distribution, "inverted_cycloid": self.inverted_cycloid_distribution, "lame_curve": self.lame_curve_distribution, "inverted_lame_curve": self.inverted_lame_curve_distribution, "linear": self.linear_distribution, "inverted_linear": self.inverted_linear_distribution, "even": self.even_distribution, "object_tracking": self.object_tracking_rotation }

        self.gui = PiSliderGUI(self.root, self)

        self.bind_keys()
        self.update_rotation_direction()
        self.on_camera_mode_change()
        if self.camera.gphoto2_available:
            self.root.after(500, self.check_camera_connection)

        self.aux_trigger_thread = threading.Thread(target=self._run_aux_trigger_listener_thread, daemon=True, name="AuxTriggerListener")
        self.aux_trigger_thread.start()

    def get_holygrail_settings_from_ui(self):
        settings_dict = {}
        try:
            for angle, data in self.holygrail_entries.items():
                ev = data['ev_var'].get()
                kelvin = data['kelvin_var'].get()
                settings_dict[angle] = [ev, kelvin]
            return settings_dict
        except Exception as e:
            logging.error(f"Could not read Holy Grail UI settings: {e}")
            messagebox.showerror("UI Error", "Could not read custom Holy Grail settings. Using defaults.")
            return None

    def start_timelapse(self):
        if self.gantry_running: return
        if not self.validate_inputs() or not self.calculate_distribution_arrays(): return

        self.gui.update_ui_for_run_state(True)
        if self.holygrail_enabled_var.get():
            self.gui.show_live_settings(True)

        self.gantry_running = True
        self.stop_flag.clear()
        self.photos_taken = 0
        self.photo_count_var.set(f"Photos: 0 / {self.num_photos_var.get()}")
        self.time_remaining_var.set("Time Rem: Estimating...")
        self.aux_trigger_fired.clear()

        self.operation_thread = threading.Thread(target=self._run_timelapse_thread, daemon=True, name="TimelapseThread")
        self.operation_thread.start()
        self.monitor_operation_thread()

    def start_preview(self):
        if self.gantry_running: return
        if not self.validate_inputs() or not self.calculate_distribution_arrays(): return
        self.gui.update_ui_for_run_state(True)
        self.gantry_running = True
        self.stop_flag.clear()
        self.operation_thread = threading.Thread(target=self._run_preview_thread, daemon=True, name="PreviewThread")
        self.operation_thread.start()
        self.monitor_operation_thread()

    def _run_timelapse_thread(self):
        try:
            if self.holygrail_enabled_var.get():
                hg_settings = self.get_holygrail_settings_from_ui()
                # --- START OF MODIFICATION: Pass all new params to controller ---
                self.holygrail_controller = HolyGrailController(
                    lat=float(self.holygrail_latitude_var.get()),
                    lon=float(self.holygrail_longitude_var.get()),
                    settings_table_dict=hg_settings,
                    day_interval_s=self.holygrail_day_interval_var.get(),
                    sunset_interval_s=self.holygrail_sunset_interval_var.get(),
                    night_interval_s=self.holygrail_night_interval_var.get(),
                    iso_transition_duration_frames=self.holygrail_iso_transition_frames_var.get()
                )
                # --- END OF MODIFICATION ---
                if self.anchor_ev is not None and self.anchor_sun_angle is not None:
                    self.holygrail_controller.anchor_ev = self.anchor_ev
                    self.holygrail_controller.anchor_sun_angle = self.anchor_sun_angle
                    logging.info(f"Applying pre-calibrated anchor: EV {self.anchor_ev:+.2f} at {self.anchor_sun_angle:.2f}°")
                else:
                    logging.warning("HG mode started without calibration.")

            length_mm = int(self.length_var.get() or "0")
            homing_dir_is_left = (self.direction_var.get() == "left")

            if length_mm > 0:
                self.hardware.move_to_end(homing_dir_is_left)
                if self.stop_flag.is_set(): return
                time.sleep(MOVEMENT_BUFFER)

            if not self.stop_flag.is_set():
                self._run_main_timelapse_loop()

            if not self.stop_flag.is_set():
                post_action = self.post_timelapse_position_var.get()
                if post_action != "Do Nothing" and length_mm > 0:
                    self.root.after(0, lambda: self.time_remaining_var.set("Post-Action: Moving..."))
                    if post_action == "Return to Start": self.hardware.move_to_end(homing_dir_is_left)
                    elif post_action == "Move to End": self.hardware.move_to_end(not homing_dir_is_left)
                    self.root.after(0, lambda: self.time_remaining_var.set("Time Rem: Done"))
        except Exception as e:
            logging.error(f"Error in timelapse thread: {e}", exc_info=True)
            self.root.after(0, lambda e=e: messagebox.showerror("Timelapse Error", str(e)))
        finally:
            self.holygrail_controller = None
            if self.root.winfo_exists():
                self.root.after(0, lambda: self.calibration_offset_var.set("Not Calibrated"))
                self.anchor_ev = None
                self.anchor_sun_angle = None
                self.root.after(0, lambda: self.gui.show_live_settings(False))

    def _run_main_timelapse_loop(self):
        total_photos = self.num_photos_var.get()
        is_hg_mode = self.holygrail_enabled_var.get()
        
        next_interval_start_time = time.time()

        for i in range(total_photos):
            if self.stop_flag.is_set(): break
            
            wait_time = next_interval_start_time - time.time()
            if wait_time > 0:
                if self.stop_flag.wait(wait_time): break
            
            if self.stop_flag.is_set(): break
            start_of_interval = time.time()

            self._perform_motor_moves(i)
            if self.stop_flag.wait(1.0): break
            
            self.photos_taken = i + 1
            self.root.after(0, lambda: self.photo_count_var.set(f"Photos: {self.photos_taken} / {total_photos}"))

            if self.aux_trigger_fired.is_set():
                logging.info(f"Shot {i+1}: Aux trigger captured photo for this interval. Skipping scheduled shot.")
                self.aux_trigger_fired.clear()
            else:
                if is_hg_mode:
                    try:
                        params = self.holygrail_controller.get_next_shot_parameters(
                            min_aperture=self.holygrail_min_aperture_var.get(),
                            max_aperture=self.holygrail_max_aperture_var.get(),
                            day_iso=self.holygrail_day_iso_var.get(),
                            night_native_iso=self.holygrail_night_iso_var.get(),
                            max_transition_iso=self.holygrail_max_transition_iso_var.get(),
                            execution_timestamp=start_of_interval
                        )
                        self.root.after(0, lambda s=params: self._update_live_settings_ui(s))
                        self.camera.apply_settings(params)
                        
                        image_path = f"/tmp/hg_shot_{i:04d}.jpg"
                        if self.camera.capture_and_download(image_path):
                             self.holygrail_controller.update_exposure_offset(image_path)
                             self.root.after(0, lambda p=image_path: self.gui.update_preview_image(p))
                        else:
                            logging.warning(f"Shot {i+1}: Failed to capture/download image, skipping reactive update.")

                    except Exception as e:
                        logging.error(f"Failed during Holy Grail shot {i+1}: {e}", exc_info=True)
                        self.root.after(0, lambda e=e: messagebox.showerror("Camera Error", f"Failed during Holy Grail shot: {e}"))
                        break
                else:
                    self.camera.trigger_photo()
                    if self.stop_flag.wait(POST_SHOT_PAUSE): break

            target_interval = 0
            if is_hg_mode and self.holygrail_controller:
                target_interval = self.holygrail_controller.last_used_interval
            else:
                target_interval = self.interval_var.get()
            
            next_interval_start_time = start_of_interval + target_interval
            rem_time_s = target_interval * (total_photos - self.photos_taken)
            self.root.after(0, lambda s=rem_time_s: self.time_remaining_var.set(f"Time Rem: ~{self.format_time(s)}"))

    def _run_preview_thread(self):
        try:
            if int(self.length_var.get() or "0") > 0:
                self.hardware.move_to_end(self.direction_var.get() == "left")
                time.sleep(MOVEMENT_BUFFER)
            total_intervals = len(self.combined_intervals)
            for i in range(total_intervals):
                if self.stop_flag.is_set(): break
                self.root.after(0, lambda i=i, t=total_intervals: self.photo_count_var.set(f"Preview Step: {i+1} / {total_intervals}"))
                self._perform_motor_moves(i)
                time.sleep(0.05)
        except Exception as e:
            logging.error(f"Error in preview thread: {e}", exc_info=True)

    def _perform_motor_moves(self, interval_index):
        self.motors_moving = True
        slider_steps, rotation_steps = 0, 0
        if interval_index < len(self.combined_intervals):
            interval_data = self.combined_intervals[interval_index]
            slider_steps = interval_data.get('slider_steps', 0)
            rotation_steps = interval_data.get('rotation_steps', 0)

            self.root.after(0, lambda s=slider_steps: self.live_slider_steps_var.set(str(s)))
            self.root.after(0, lambda r=rotation_steps: self.live_rotation_steps_var.set(str(r)))

            self.hardware.enable_motors()
            if slider_steps > 0: self.hardware.set_slider_step(slider_steps, self.direction_var.get() == "right")
            if self.stop_flag.is_set():
                self.motors_moving = False; self.hardware.disable_motors(); return
            if rotation_steps != 0:
                is_forward = (self.hardware.rotation_direction_gpio == GPIO.HIGH) != (rotation_steps < 0)
                self.hardware.move_steps(ROTATION_EN_PIN, ROTATION_DIR_PIN, ROTATION_STEP_PIN, abs(rotation_steps), is_forward)
            self.hardware.disable_motors()

        if interval_index >= len(self.combined_intervals) - 1:
            self.root.after(100, lambda: self.live_slider_steps_var.set("--"))
            self.root.after(100, lambda: self.live_rotation_steps_var.set("--"))
        self.motors_moving = False

    def _run_aux_trigger_listener_thread(self):
        logging.info("Auxiliary trigger listener thread started.")
        while not self.stop_flag.is_set():
            if self.gantry_running and self.holygrail_enabled_var.get():
                if GPIO.input(AUX_TRIGGER_PIN) == GPIO.LOW:
                    if not self.aux_trigger_fired.is_set():
                        logging.info("Auxiliary trigger detected by hardware! Firing camera and setting flag.")
                        self.camera.trigger_photo()
                        self.aux_trigger_fired.set()
            time.sleep(0.05)
        logging.info("Auxiliary trigger listener thread stopped.")

    def monitor_operation_thread(self):
        if self.operation_thread and self.operation_thread.is_alive():
            self.root.after(100, self.monitor_operation_thread)
        else:
            self.gantry_running = False; self.motors_moving = False
            if self.root.winfo_exists():
                self.gui.update_ui_for_run_state(False)
                self.time_remaining_var.set("Time Rem: Finished")

    def on_closing(self):
        if self.gantry_running:
            if messagebox.askyesno("Confirm Exit", "Operation in progress. Stop and exit?"):
                self.stop_flag.set()
                if self.operation_thread: self.operation_thread.join(timeout=5.0)
                if self.aux_trigger_thread: self.aux_trigger_thread.join(timeout=1.0)
                self.cleanup_and_exit_app()
        else:
            self.cleanup_and_exit_app()

    def cleanup_and_exit_app(self):
        self.stop_flag.set()
        if self.aux_trigger_thread and self.aux_trigger_thread.is_alive():
             self.aux_trigger_thread.join(timeout=1.0)
        if self.camera: self.camera.cleanup()
        if self.root.winfo_exists(): self.root.destroy()
        logging.info("--- PiSlider Application Shutdown ---")

    def request_stop_operations(self):
        self.stop_flag.set()
        if self.gui.stop_button.winfo_exists(): self.gui.stop_button.config(state=tkinter.DISABLED)

    def on_entry_focus(self, event): self.entry_has_focus = True
    def on_entry_defocus(self, event): self.entry_has_focus = False

    def bind_keys(self): self.root.bind("<Key>", self.handle_key_event)

    def handle_key_event(self, event):
        if self.entry_has_focus: return
        key_map = { "KP_Multiply": self.rotate_ccw_manual, "asterisk": self.rotate_ccw_manual, "KP_Divide": self.rotate_cw_manual, "slash": self.rotate_cw_manual, "KP_Add": self.gantry_right_manual, "plus": self.gantry_right_manual, "equal": self.gantry_right_manual, "KP_Subtract": self.gantry_left_manual, "minus": self.gantry_left_manual, "KP_Enter": self.manual_trigger_action, "Return": self.manual_trigger_action }
        action = key_map.get(event.keysym)
        if action:
            if action == self.manual_trigger_action: action()
            elif not self.gantry_running: action()
            else: logging.warning(f"Key action ignored, gantry running.")

    def manual_trigger_action(self):
        if self.mode_var.get() == 'manual' and self.gantry_running and not self.motors_moving: self.force_interval = True

    def _manual_move_wrapper(self, move_func, *args):
        if self.gantry_running or self.motors_moving: return
        threading.Thread(target=move_func, args=args, daemon=True).start()

    def rotate_cw_manual(self): self._manual_move_wrapper(self.hardware.rotate_motor, 1, True)
    def rotate_ccw_manual(self): self._manual_move_wrapper(self.hardware.rotate_motor, 1, False)
    def gantry_right_manual(self): self._manual_move_wrapper(self.hardware.set_slider_step, 50, True)
    def gantry_left_manual(self): self._manual_move_wrapper(self.hardware.set_slider_step, 50, False)

    def update_rotation_direction(self, event=None):
        self.hardware.rotation_direction_gpio = GPIO.HIGH if self.rotation_direction_var.get() == "CW" else GPIO.LOW

    def update_mode(self): self.gui.update_mode_widgets()

    def toggle_holygrail_mode(self):
        self.gui.toggle_holygrail_widgets()
        if self.holygrail_enabled_var.get():
            self.camera_trigger_mode_var.set("USB Control")
            if self.gui.camera_trigger_combo: self.gui.camera_trigger_combo.config(state=tkinter.DISABLED)
        else:
            if self.gui.camera_trigger_combo: self.gui.camera_trigger_combo.config(state="readonly")
        self.on_camera_mode_change()

    def check_camera_connection(self):
        if not self.camera.gphoto2_available:
            self.camera_status_var.set("Status: gphoto2 not installed.")
            return
        if self.camera_check_thread and self.camera_check_thread.is_alive():
            logging.info("Camera check already in progress.")
            return

        def _threaded_check():
            self.root.after(0, lambda: self.gui.check_camera_button.config(state=tkinter.DISABLED))
            self.root.after(0, lambda: self.camera_status_var.set("Status: Checking..."))

            summary = self.camera.get_camera_summary()

            self.root.after(0, lambda s=summary: self.camera_status_var.set(f"Status: {s}"))
            if self.root.winfo_exists():
                if "Connected" in summary:
                    self.root.after(0, lambda: self.gui.camera_status_label.config(foreground='green'))
                else:
                    self.root.after(0, lambda: self.gui.camera_status_label.config(foreground='red'))
                self.root.after(0, lambda: self.gui.check_camera_button.config(state=tkinter.NORMAL))

        self.camera_check_thread = threading.Thread(target=_threaded_check, daemon=True, name="CameraCheckThread")
        self.camera_check_thread.start()

    def on_camera_mode_change(self, event=None):
        self.camera.set_trigger_mode(self.camera_trigger_mode_var.get())
        is_usb = self.camera_trigger_mode_var.get() == "USB Control"
        usb_state = tkinter.NORMAL if is_usb and self.camera.gphoto2_available else tkinter.DISABLED
        if self.gui.take_preview_button: self.gui.take_preview_button.config(state=usb_state)
        if self.gui.check_camera_button: self.gui.check_camera_button.config(state=usb_state)
        if self.gui.calibrate_button: self.gui.calibrate_button.config(state=tkinter.NORMAL if is_usb and self.holygrail_enabled_var.get() else tkinter.DISABLED)
        if not is_usb:
            self.camera_status_var.set("Status: S2 Cable Mode")
            if self.gui.camera_status_label: self.gui.camera_status_label.config(foreground='blue')
        else:
            self.camera_status_var.set("Status: USB Mode (Unknown Connection)")
            if self.gui.camera_status_label: self.gui.camera_status_label.config(foreground='black' if is_usb else 'blue')

    def calibrate_holygrail_exposure(self):
        if not self.camera.gphoto2_available or self.camera.trigger_mode != 'USB Control':
            messagebox.showwarning("Calibration Error", "Calibration requires USB Control mode.")
            return
        if self.operation_thread and self.operation_thread.is_alive():
            messagebox.showwarning("Busy", "Another camera operation is in progress.")
            return

        def run_calib():
            logging.info("Calibration process started")
            self.root.after(0, lambda: self.gui.update_ui_for_run_state(True))
            self.root.after(0, lambda: self.calibration_offset_var.set("Reading settings..."))
            try:
                iso_str = self.camera.get_config_value("iso")
                aperture_str = self.camera.get_config_value("f-number")
                shutter_str = self.camera.get_config_value("shutterspeed")
                user_iso = int(iso_str)
                user_aperture = float(aperture_str[2:]) if aperture_str.startswith('f/') else float(aperture_str)
                user_shutter_s = self.camera.get_shutter_s(shutter_str)
            except Exception as e:
                logging.error(f"Could not read camera settings for calibration: {e}")
                self.root.after(0, lambda: self.calibration_offset_var.set("Read Failed")); self.root.after(0, lambda: self.gui.update_ui_for_run_state(False)); return

            self.root.after(0, lambda: self.calibration_offset_var.set("Capturing..."))
            image_path = "/tmp/hg_calib.jpg"
            if self.camera.capture_and_download(image_path):
                self.root.after(0, lambda p=image_path: self.gui.update_preview_image(p))
                hg_settings = self.get_holygrail_settings_from_ui()
                controller = HolyGrailController(float(self.holygrail_latitude_var.get()), float(self.holygrail_longitude_var.get()), hg_settings, self.holygrail_day_interval_var.get(), self.holygrail_sunset_interval_var.get(), self.holygrail_night_interval_var.get())
                if controller.calibrate(image_path, user_iso, user_aperture, user_shutter_s):
                    self.anchor_ev = controller.anchor_ev
                    self.anchor_sun_angle = controller.anchor_sun_angle
                    self.root.after(0, lambda: self.calibration_offset_var.set(f"Anchor EV: {self.anchor_ev:+.2f}"))
            else:
                self.root.after(0, lambda: self.calibration_offset_var.set("Capture Failed"))
            self.root.after(0, lambda: self.gui.update_ui_for_run_state(False))

        self.preview_thread = threading.Thread(target=run_calib, daemon=True); self.preview_thread.start()

    def take_preview(self):
        if not self.camera.gphoto2_available or self.camera.trigger_mode != 'USB Control':
            messagebox.showwarning("Preview Error", "Preview requires USB Control mode.")
            return
        if self.preview_thread and self.preview_thread.is_alive():
            messagebox.showwarning("Busy", "Another camera operation is in progress.")
            return

        def run_preview():
            self.root.after(0, lambda: self.gui.take_preview_button.config(state=tkinter.DISABLED))
            preview_path = "/tmp/ui_preview.jpg"
            if self.camera.capture_and_download(preview_path):
                self.root.after(0, lambda p=preview_path: self.gui.update_preview_image(p))
            else:
                self.root.after(0, lambda: messagebox.showerror("Preview Failed", "Could not capture the preview image."))
            if self.root.winfo_exists():
                self.root.after(0, lambda: self.gui.take_preview_button.config(state=tkinter.NORMAL))

        self.preview_thread = threading.Thread(target=run_preview, daemon=True); self.preview_thread.start()

    def validate_inputs(self):
        try:
            int(self.length_var.get() or "0"); float(self.rotation_angle_var.get() or "0.0")
            if self.num_photos_var.get() <= 0: raise ValueError("Num photos must be > 0.")
            if self.holygrail_enabled_var.get():
                float(self.holygrail_latitude_var.get()); float(self.holygrail_longitude_var.get())
                if self.holygrail_min_aperture_var.get() <= 0 or self.holygrail_max_aperture_var.get() <= 0 or self.holygrail_day_iso_var.get() <= 0 or self.holygrail_night_iso_var.get() <= 0 or self.holygrail_max_transition_iso_var.get() <= 0 or self.holygrail_iso_transition_frames_var.get() <= 0:
                    raise ValueError("Aperture, ISO, and Transition Frame values must be positive.")
            return True
        except (ValueError, tkinter.TclError) as e:
            messagebox.showerror("Input Error", f"Invalid input value: {e}"); return False

    def calculate_distribution_arrays(self):
        try:
            num_photos = self.num_photos_var.get(); length_mm = int(self.length_var.get() or "0"); min_speed_mm = self.minimum_slider_speed_var.get()
            slider_micro = self.hardware.slider_microstepping if self.hardware else 4
            dist_pulse = (BELT_PITCH * PULLEY_TEETH) / (STEPS_PER_REVOLUTION_SLIDER * slider_micro)
            self.total_steps = int(length_mm / dist_pulse)
            min_speed_pulses = int(min_speed_mm / dist_pulse) if dist_pulse > 0 else 0
            total_min_speed_pulses = min_speed_pulses * num_photos
            slider_func = self.distribution_functions.get(self.distribution_var.get(), self.even_distribution)
            if total_min_speed_pulses >= self.total_steps and self.total_steps > 0:
                self.steps_distribution = self.scale_array_to_sum(slider_func(num_photos), self.total_steps)
            else:
                base_distribution = self.scale_array_to_sum(slider_func(num_photos), self.total_steps - total_min_speed_pulses)
                self.steps_distribution = base_distribution + min_speed_pulses
            if self.total_steps > 0 and self.steps_distribution.size > 0 and np.sum(self.steps_distribution) != self.total_steps: self.steps_distribution[-1] += self.total_steps - np.sum(self.steps_distribution)
            rot_angle = self.rotation_angle_var.get()
            total_rot_pulses = int((rot_angle / 360.0) * STEPS_PER_REVOLUTION_ROTATION)
            rot_func = self.distribution_functions.get(self.rotation_distribution_var.get())
            value_for_rot_func = rot_angle if self.rotation_distribution_var.get() == "object_tracking" else total_rot_pulses
            rot_raw = rot_func(num_photos, value_for_rot_func)
            rot_final = self.scale_array_to_sum(rot_raw, total_rot_pulses) if self.rotation_distribution_var.get() != "object_tracking" else rot_raw.astype(int)
            if total_rot_pulses != 0 and rot_final.size > 0 and np.sum(rot_final) != total_rot_pulses: rot_final[-1] += total_rot_pulses - np.sum(rot_final)
            self.combined_intervals = [{'slider_steps': int(s), 'rotation_steps': int(r)} for s, r in zip(self.steps_distribution, rot_final)]
            return True
        except Exception as e:
            messagebox.showerror("Calculation Error", f"Failed to calculate distributions: {e}"); return False

    def scale_array_to_sum(self, arr_in, target_sum_val):
        arr = np.nan_to_num(np.array(arr_in, dtype=float)); target_sum = float(target_sum_val)
        if arr.size == 0: return np.array([], dtype=int)
        current_sum = np.sum(arr)
        if abs(current_sum) < 1e-9:
            if arr.size > 0 and target_sum != 0:
                base, rem = divmod(int(round(target_sum)), arr.size)
                return np.full(arr.size, base, dtype=int) + np.array([1]*rem + [0]*(arr.size-rem))
            return np.zeros(arr.size, dtype=int)
        scaled_float = arr * (target_sum / current_sum)
        int_arr = np.floor(scaled_float).astype(int)
        error_to_dist = int(round(target_sum)) - np.sum(int_arr)
        if error_to_dist > 0:
            indices = np.argsort(scaled_float - int_arr)[-error_to_dist:]
            int_arr[indices] += 1
        return int_arr.astype(int)

    def format_time(self, seconds):
        if not isinstance(seconds, (int, float)) or seconds < 0: return "--:--"
        m, s = divmod(int(seconds), 60); h, m = divmod(m, 60)
        return f"{h:02d}:{m:02d}:{s:02d}" if h > 0 else f"{m:02d}:{s:02d}"

    def _update_live_settings_ui(self, settings):
        self.live_iso_var.set(str(settings['iso']))
        self.live_shutter_var.set(str(settings['shutter']) if settings['mode'] != 'bulb' else f"{settings['bulb_duration']:.1f}s (Bulb)")
        self.live_aperture_var.set(str(settings['aperture']))
        self.live_kelvin_var.set(str(settings['kelvin']))
        self.live_ev_offset_var.set(f"{settings.get('ev_offset', 0):+.2f} EV")
        self.live_interval_var.set(f"{settings.get('target_interval', 0):.1f}s")

    def catenary_distribution(self, total_intervals, *args):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-1.5, 1.5, total_intervals); curve = np.cosh(x)
        return curve - np.min(curve)

    def inverted_catenary_distribution(self, total_intervals, *args):
        s = self.catenary_distribution(total_intervals)
        return (np.max(s) if s.size > 0 else s) - s

    def gaussian_distribution(self, total_intervals, *args):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-2.5, 2.5, total_intervals); curve = np.exp(-x**2 / 2)
        return curve - np.min(curve)

    def inverted_gaussian_distribution(self, total_intervals, *args):
        s = self.gaussian_distribution(total_intervals)
        return (np.max(s) if s.size > 0 else s) - s

    def ellipsoidal_distribution(self, total_intervals, *args):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-1, 1, total_intervals); curve = (1 - x**2)**0.5; curve[np.isnan(curve)] = 0
        return curve - np.min(curve)

    def inverted_ellipsoidal_distribution(self, total_intervals, *args):
        s = self.ellipsoidal_distribution(total_intervals)
        return (np.max(s) if s.size > 0 else s) - s

    def parabolic_distribution(self, total_intervals, *args):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-1, 1, total_intervals); curve = 1 - x**2
        return curve - np.min(curve)

    def inverted_parabolic_distribution(self, total_intervals, *args):
        s = self.parabolic_distribution(total_intervals)
        return (np.max(s) if s.size > 0 else s) - s

    def cycloid_distribution(self, total_intervals, *args):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(0, 2 * np.pi, total_intervals); curve = (1 - np.cos(x)) / 2
        return curve - np.min(curve)

    def inverted_cycloid_distribution(self, total_intervals, *args):
        s = self.cycloid_distribution(total_intervals)
        return (np.max(s) if s.size > 0 else s) - s

    def lame_curve_distribution(self, total_intervals, *args, n_param=4):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-1, 1, total_intervals); curve = (1 - np.abs(x)**n_param)**(1/n_param); curve[np.isnan(curve)] = 0
        return curve - np.min(curve)

    def inverted_lame_curve_distribution(self, total_intervals, *args):
        s = self.lame_curve_distribution(total_intervals)
        return (np.max(s) if s.size > 0 else s) - s

    def linear_distribution(self, total_intervals, *args):
        return np.linspace(0.01, 1, total_intervals) if total_intervals > 0 else np.array([])

    def inverted_linear_distribution(self, total_intervals, *args):
        return np.linspace(1, 0.01, total_intervals) if total_intervals > 0 else np.array([])

    def even_distribution(self, total_intervals, *args):
        return np.ones(total_intervals) if total_intervals > 0 else np.array([])

    def object_tracking_rotation(self, total_intervals, total_rotation_angle_deg, *args):
        if total_intervals <= 0 or self.steps_distribution.size != total_intervals: return np.zeros(total_intervals)
        if self.total_steps == 0:
            angle_pulses = int((total_rotation_angle_deg / 360.0) * STEPS_PER_REVOLUTION_ROTATION)
            return self.scale_array_to_sum(np.ones(total_intervals), angle_pulses)
        dist_per_pulse = (BELT_PITCH * PULLEY_TEETH) / (STEPS_PER_REVOLUTION_SLIDER * self.hardware.slider_microstepping)
        slider_move_mm_interval = self.steps_distribution * dist_per_pulse
        total_slider_travel_mm = self.total_steps * dist_per_pulse
        if abs(total_rotation_angle_deg) < 1e-3: return np.zeros(total_intervals)
        dist_to_obj_mm = (total_slider_travel_mm / 2.0) / math.tan(math.radians(abs(total_rotation_angle_deg)) / 2.0)
        cumulative_slider_pos_mm = 0.0
        initial_x_offset = -total_slider_travel_mm / 2.0
        previous_angle_rad = math.atan2(initial_x_offset, dist_to_obj_mm)
        rotation_angles_rad_interval = np.zeros(total_intervals)
        for i in range(total_intervals):
            cumulative_slider_pos_mm += slider_move_mm_interval[i]
            current_x_offset = cumulative_slider_pos_mm - (total_slider_travel_mm / 2.0)
            target_angle_rad = math.atan2(current_x_offset, dist_to_obj_mm)
            rotation_angles_rad_interval[i] = target_angle_rad - previous_angle_rad
            previous_angle_rad = target_angle_rad
        pulses_per_radian = STEPS_PER_REVOLUTION_ROTATION / (2 * math.pi)
        return rotation_angles_rad_interval * pulses_per_radian

if __name__ == "__main__":
    def main():
        root = None
        try:
            root = tkinter.Tk()
            try: root.tk.call('tk', 'scaling', 2.0)
            except tkinter.TclError: logging.warning("Could not set Tk scaling.")
            root.title("PiSlider Control")
            app = PiSliderApp(root)
            if app.hardware and root.winfo_exists(): root.mainloop()
        except Exception as e:
            logging.critical(f"A fatal error occurred in main: {e}", exc_info=True)
            if root and root.winfo_exists(): messagebox.showerror("Fatal Error", f"A critical error occurred: {e}")
        finally:
            logging.info("--- PiSlider Application Shutdown ---")
    main()
