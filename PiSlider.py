import threading
import time
import tkinter
from tkinter import messagebox
import numpy as np
import RPi.GPIO as GPIO
import os
import math
import logging
import csv
from datetime import datetime

# Local project imports
from gui import PiSliderGUI
from holygrail import HolyGrailController
from hardware import HardwareController, CameraController, ROTATION_EN_PIN, ROTATION_DIR_PIN, ROTATION_STEP_PIN, AUX_TRIGGER_PIN

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s')

# --- Constants ---
BELT_PITCH = 2
PULLEY_TEETH = 16
STEPS_PER_REVOLUTION_SLIDER = 200 * 2
STEPS_PER_REVOLUTION_ROTATION = 48000
MOVEMENT_BUFFER = 2
POST_SHOT_PAUSE = 1.0

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
            if self.root.winfo_exists(): self.root.destroy(); return

        # --- Initialize all state variables ---
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
        self.anchor_offset_ev = 0.0
        self.report_file, self.report_writer = None, None
        self.last_xmp_offset = 0.0
        self.last_raw_filepath = ""

        # --- Tkinter Model Variables ---
        self.save_path_var = tkinter.StringVar()
        self.direction_var = tkinter.StringVar(value='right')
        self.length_var = tkinter.StringVar(value="2800")
        self.distribution_var = tkinter.StringVar(value="even")
        self.rotation_angle_var = tkinter.DoubleVar(value=90.0)
        self.rotation_direction_var = tkinter.StringVar(value="CW")
        self.rotation_distribution_var = tkinter.StringVar(value="even")
        self.num_photos_var = tkinter.IntVar(value=360)
        self.interval_var = tkinter.DoubleVar(value=8.0)
        self.minimum_slider_speed_var = tkinter.IntVar(value=0)
        self.mode_var = tkinter.StringVar(value='timed')
        self.post_timelapse_position_var = tkinter.StringVar(value="Do Nothing")
        self.camera_trigger_mode_var = tkinter.StringVar(value="USB Control")
        self.camera_status_var = tkinter.StringVar(value="Status: Unknown")
        self.photo_count_var = tkinter.StringVar(value="Photos: 0 / 0")
        self.time_remaining_var = tkinter.StringVar(value="Time Rem: --:--")
        self.holygrail_enabled_var = tkinter.BooleanVar(value=True)
        self.holygrail_latitude_var = tkinter.StringVar(value="48.17293")
        self.holygrail_longitude_var = tkinter.StringVar(value="-82.521721")
        self.holygrail_timezone_var = tkinter.StringVar(value="America/Toronto")
        self.holygrail_min_aperture_var = tkinter.DoubleVar(value=2.8)
        self.holygrail_max_aperture_var = tkinter.DoubleVar(value=11.0)
        self.holygrail_day_iso_var = tkinter.IntVar(value=100)
        self.holygrail_night_iso_var = tkinter.IntVar(value=640)
        self.holygrail_max_transition_iso_var = tkinter.IntVar(value=5000)
        self.holygrail_iso_transition_frames_var = tkinter.IntVar(value=30)
        self.holygrail_day_interval_var = tkinter.DoubleVar(value=8.0)
        self.holygrail_sunset_interval_var = tkinter.DoubleVar(value=5.0)
        self.holygrail_night_interval_var = tkinter.DoubleVar(value=40.0)
        self.calibration_offset_var = tkinter.StringVar(value="Not Calibrated")
        self.live_sun_angle_var = tkinter.StringVar(value="--")
        self.live_iso_var, self.live_shutter_var, self.live_aperture_var, self.live_kelvin_var, self.live_ev_offset_var = (tkinter.StringVar(value="--") for _ in range(5))
        self.live_interval_var, self.live_slider_steps_var, self.live_rotation_steps_var = (tkinter.StringVar(value="--") for _ in range(3))

        self.entry_widgets, self.preview_photo_image, self.distribution_photo_image, self.holygrail_entries = [], None, None, {}
        self.distribution_images = { "catenary": "catenary.png", "inverted_catenary": "inverted_catenary.png", "gaussian": "gaussian.png", "inverted_gaussian": "inverted_gaussian.png", "ellipsoidal": "ellipsoidal.png", "inverted_ellipsoidal": "ellipsoidal.png", "parabolic": "parabolic.png", "inverted_parabolic": "inverted_parabolic.png", "cycloid": "cycloid.png", "inverted_cycloid": "inverted_cycloid.png", "lame_curve": "lame_curve.png", "inverted_lame_curve": "inverted_lame_curve.png", "linear": "linear.png", "inverted_linear": "linear.png", "even": "even.png" }
        self.distribution_functions = { "catenary": self.catenary_distribution, "inverted_catenary": self.inverted_catenary_distribution, "gaussian": self.gaussian_distribution, "inverted_gaussian": self.inverted_gaussian_distribution, "ellipsoidal": self.ellipsoidal_distribution, "inverted_ellipsoidal": self.inverted_ellipsoidal_distribution, "parabolic": self.parabolic_distribution, "inverted_parabolic": self.inverted_parabolic_distribution, "cycloid": self.cycloid_distribution, "inverted_cycloid": self.inverted_cycloid_distribution, "lame_curve": self.lame_curve_distribution, "inverted_lame_curve": self.inverted_lame_curve_distribution, "linear": self.linear_distribution, "inverted_linear": self.inverted_linear_distribution, "even": self.even_distribution, "object_tracking": self.object_tracking_rotation }

        self.gui = PiSliderGUI(self.root, self)
        self.bind_keys()
        self.update_rotation_direction()
        self.on_camera_mode_change()
        self.detect_storage_paths()
        
        self.aux_trigger_thread = threading.Thread(target=self._run_aux_trigger_listener_thread, daemon=True, name="AuxTriggerListener")
        self.aux_trigger_thread.start()

    def detect_storage_paths(self):
        pi_user = os.getlogin() if hasattr(os, 'getlogin') else 'pi'
        search_paths = [f'/media/{pi_user}/', '/mnt/']
        found_drives = []
        for path in search_paths:
            if os.path.exists(path):
                try:
                    drives = [os.path.join(path, d) for d in os.listdir(path) if os.path.isdir(os.path.join(path, d))]
                    found_drives.extend(drives)
                except Exception as e:
                    logging.warning(f"Could not scan {path} for drives: {e}")
        fallback_path = os.path.expanduser("~/PiSlider_Captures")
        if not found_drives:
            found_drives.append(fallback_path)
            logging.info(f"No external storage detected. Using fallback path: {fallback_path}")
        else:
            logging.info(f"Detected external drives: {found_drives}")
        if self.gui.save_path_combo:
            self.gui.save_path_combo['values'] = found_drives
            self.save_path_var.set(found_drives[0])
    
    def _create_xmp_sidecar(self, image_path, exposure_offset, kelvin):
        xmp_path = os.path.splitext(image_path)[0] + ".xmp"
        exposure_tag = f'<crs:Exposure2012>{exposure_offset:+.2f}</crs:Exposure2012>' if abs(exposure_offset) >= 0.01 else ''
        xmp_content = f"""<x:xmpmeta xmlns:x="adobe:ns:meta/" x:xmptk="Adobe XMP Core 5.6-c140 79.160451, 2017/05/06-01:08:21        ">
 <rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">
  <rdf:Description rdf:about="" xmlns:crs="http://ns.adobe.com/camera-raw-settings/1.0/">
   {exposure_tag}
   <crs:WhiteBalance>Custom</crs:WhiteBalance>
   <crs:Temperature>{kelvin}</crs:Temperature>
  </rdf:Description>
 </rdf:RDF>
</x:xmpmeta>"""
        try:
            with open(xmp_path, 'w') as f: f.write(xmp_content)
            logging.info(f"Created XMP for {os.path.basename(image_path)} with EV offset: {exposure_offset:+.2f}, Kelvin: {kelvin}")
        except Exception as e: logging.error(f"Failed to write XMP file {xmp_path}: {e}")

    def get_holygrail_settings_from_ui(self):
        settings_dict = {}
        try:
            for angle, data in self.holygrail_entries.items():
                settings_dict[angle] = [data['ev_var'].get(), data['kelvin_var'].get()]
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
            try:
                session_name = f'timelapse_{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}'
                session_path = os.path.join(self.save_path_var.get(), session_name)
                os.makedirs(session_path, exist_ok=True)
                self.save_path_var.set(session_path)
                report_dir = os.path.join(session_path, 'reports')
                os.makedirs(report_dir, exist_ok=True)
                filename = os.path.join(report_dir, f'hg_report_{session_name}.csv')
                self.report_file = open(filename, 'w', newline='')
                self.report_writer = csv.writer(self.report_file)
                header = ['Timestamp', 'Frame', 'Sun_Elevation_Deg', 'Ideal_Target_EV', 'Reactive_EV_Offset', 'Final_Target_EV', 'Actual_Settings_EV', 'XMP_EV_Offset', 'Is_Aux_Trigger_Replacement', 'Is_ISO_Transition_Frame', 'Final_ISO', 'Final_Shutter_Str', 'Final_Bulb_Duration_s', 'Final_Aperture', 'Final_Kelvin', 'Interval_Blended_s', 'Interval_Required_s', 'Interval_Final_s', 'Slider_Steps', 'Rotation_Steps']
                self.report_writer.writerow(header)
                logging.info(f"Holy Grail report created at: {filename}")
            except Exception as e:
                logging.error(f"Failed to create session or report file: {e}"); self.report_file = None; self.report_writer = None
        
        self.gantry_running, self.photos_taken = True, 0
        self.stop_flag.clear(); self.aux_trigger_fired.clear()
        self.photo_count_var.set(f"Photos: 0 / {self.num_photos_var.get()}"); self.time_remaining_var.set("Time Rem: Estimating...")
        self.operation_thread = threading.Thread(target=self._run_timelapse_thread, daemon=True, name="TimelapseThread"); self.operation_thread.start()
        self.monitor_operation_thread()

    def start_preview(self):
        if self.gantry_running: logging.warning("Cannot start preview, an operation is already running."); return
        if not self.validate_inputs() or not self.calculate_distribution_arrays(): return
        self.gui.update_ui_for_run_state(True)
        self.gantry_running = True
        self.stop_flag.clear()
        self.photo_count_var.set(f"Previewing {self.num_photos_var.get()} steps...")
        self.time_remaining_var.set("Time Rem: --:--")
        self.operation_thread = threading.Thread(target=self._run_preview_thread, daemon=True, name="PreviewThread")
        self.operation_thread.start()
        self.monitor_operation_thread()

    def _run_timelapse_thread(self):
        try:
            if self.holygrail_enabled_var.get():
                self.holygrail_controller = HolyGrailController(
                    lat=float(self.holygrail_latitude_var.get()),
                    lon=float(self.holygrail_longitude_var.get()),
                    settings_table_dict=self.get_holygrail_settings_from_ui(),
                    day_interval_s=self.holygrail_day_interval_var.get(),
                    sunset_interval_s=self.holygrail_sunset_interval_var.get(),
                    night_interval_s=self.holygrail_night_interval_var.get(),
                    timezone_str=self.holygrail_timezone_var.get(),
                    iso_transition_duration_frames=self.holygrail_iso_transition_frames_var.get()
                )
                if self.anchor_offset_ev != 0.0:
                    self.holygrail_controller.anchor_offset = self.anchor_offset_ev
                    logging.info(f"Applying pre-calibrated anchor offset: {self.anchor_offset_ev:+.2f} EV")
            
            if int(self.length_var.get() or "0") > 0:
                self.hardware.move_to_end(self.direction_var.get() == "left")
                if self.stop_flag.is_set(): return
            
            if not self.stop_flag.is_set(): self._run_main_timelapse_loop()
        except Exception as e:
            logging.error(f"Error in timelapse thread: {e}", exc_info=True)
            self.root.after(0, lambda e=e: messagebox.showerror("Timelapse Error", str(e)))
        finally:
            self.gantry_running = False
            self.holygrail_controller = None
            if self.root.winfo_exists():
                self.root.after(0, lambda: self.calibration_offset_var.set("Not Calibrated"))
                self.anchor_offset_ev = 0.0
                self.root.after(0, lambda: self.gui.show_live_settings(False))
            if self.report_file:
                try: self.report_file.close(); logging.info("Report file closed.")
                except Exception as e: logging.error(f"Error closing report file: {e}")
                self.report_file, self.report_writer = None, None
            self.detect_storage_paths()

    def _run_main_timelapse_loop(self):
        total_photos = self.num_photos_var.get()
        is_hg_mode = self.holygrail_enabled_var.get()
        
        next_interval_start_time = time.time()

        for i in range(total_photos):
            if self.stop_flag.is_set(): break
            
            wait_time = next_interval_start_time - time.time()
            if wait_time > 0:
                if self.stop_flag.wait(wait_time):
                    break
            
            if self.stop_flag.is_set(): break
            
            start_of_interval = time.time()
            
            report_data = {'Timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'), 'Frame': i + 1}
            self._perform_motor_moves(i)
            report_data.update({'Slider_Steps': self.combined_intervals[i]['slider_steps'], 'Rotation_Steps': self.combined_intervals[i]['rotation_steps']})

            if self.stop_flag.wait(1.0): break
            
            self.photos_taken = i + 1
            self.root.after(0, lambda: self.photo_count_var.set(f"Photos: {self.photos_taken} / {total_photos}"))

            if self.aux_trigger_fired.is_set():
                logging.info(f"Shot {i+1}: Aux trigger captured photo. Skipping scheduled shot.")
                self.aux_trigger_fired.clear()
                report_data['Is_Aux_Trigger_Replacement'] = 'Yes'
            else:
                report_data['Is_Aux_Trigger_Replacement'] = 'No'
                if is_hg_mode:
                    try:
                        params = self.holygrail_controller.get_next_shot_parameters(
                            self.holygrail_min_aperture_var.get(), self.holygrail_max_aperture_var.get(),
                            self.holygrail_day_iso_var.get(), self.holygrail_night_iso_var.get(),
                            self.holygrail_max_transition_iso_var.get(), start_of_interval)
                        
                        sun_angle = self.holygrail_controller.last_sun_elevation
                        self.root.after(0, lambda sa=sun_angle: self.live_sun_angle_var.set(f"{sa:.2f}°"))
                        
                        xmp_offset = params.get('xmp_ev_offset', 0.0)
                        self.last_xmp_offset = xmp_offset
                        
                        full_image_path = os.path.join(self.save_path_var.get(), f"Frame_{i:04d}.ARW")
                        self.last_raw_filepath = full_image_path
                        
                        preview_path = f"/tmp/hg_preview_{i:04d}.jpg"
                        
                        self.camera.apply_settings(params)
                        self.root.after(0, lambda s=params: self._update_live_settings_ui(s))
                        self._create_xmp_sidecar(full_image_path, xmp_offset, params['kelvin'])
                        
                        if self.camera.capture_and_download(full_image_path, preview_path):
                             self.holygrail_controller.update_exposure_offset(preview_path, xmp_offset)
                             self.root.after(0, lambda p=preview_path: self.gui.update_preview_image(p))
                        
                        hg = self.holygrail_controller
                        ideal_ev = np.interp(hg.last_sun_elevation, hg.ev_sun_angles, hg.target_evs)
                        
                        # Corrected np.interp argument order
                        blended_s = np.interp(hg.last_sun_elevation, hg.interval_sun_angles, hg.target_intervals)

                        report_data.update({
                            'Sun_Elevation_Deg': f"{hg.last_sun_elevation:.4f}", 'Ideal_Target_EV': f"{ideal_ev:+.4f}",
                            'Reactive_EV_Offset': f"{hg.reactive_ev_offset:+.4f}", 'Final_Target_EV': f"{ideal_ev + hg.anchor_offset + hg.reactive_ev_offset:+.4f}",
                            'Actual_Settings_EV': f"{params.get('actual_settings_ev', 0):+.4f}", 'XMP_EV_Offset': f"{xmp_offset:+.4f}",
                            'Is_ISO_Transition_Frame': 'Yes' if hg.is_in_iso_transition else 'No', 'Final_ISO': params['iso'], 'Final_Shutter_Str': params['shutter'],
                            'Final_Bulb_Duration_s': f"{params['bulb_duration']:.2f}", 'Final_Aperture': params['aperture'], 'Final_Kelvin': params['kelvin'],
                            'Interval_Blended_s': f"{blended_s:.2f}",
                            'Interval_Required_s': f"{hg.last_exposure_duration + hg.MIN_INTERVAL_SAFETY_BUFFER_S:.2f}", 'Interval_Final_s': f"{params['target_interval']:.2f}"})
                    except Exception as e: logging.error(f"HG shot {i+1} failed: {e}", exc_info=True); break
                else:
                    self.camera.trigger_photo(); self.stop_flag.wait(POST_SHOT_PAUSE)
            
            if self.report_writer:
                header = ['Timestamp', 'Frame', 'Sun_Elevation_Deg', 'Ideal_Target_EV', 'Reactive_EV_Offset', 'Final_Target_EV', 'Actual_Settings_EV', 'XMP_EV_Offset', 'Is_Aux_Trigger_Replacement', 'Is_ISO_Transition_Frame', 'Final_ISO', 'Final_Shutter_Str', 'Final_Bulb_Duration_s', 'Final_Aperture', 'Final_Kelvin', 'Interval_Blended_s', 'Interval_Required_s', 'Interval_Final_s', 'Slider_Steps', 'Rotation_Steps']
                self.report_writer.writerow([report_data.get(h, '') for h in header])

            target_interval = (self.holygrail_controller.last_used_interval if is_hg_mode and self.holygrail_controller else self.interval_var.get())
            
            next_interval_start_time += target_interval
            
            rem_time_s = target_interval * (total_photos - self.photos_taken)
            self.root.after(0, lambda s=rem_time_s: self.time_remaining_var.set(f"Time Rem: ~{self.format_time(s)}"))

    def _run_preview_thread(self):
        try:
            if int(self.length_var.get() or "0") > 0:
                self.hardware.move_to_end(self.direction_var.get() == "left"); time.sleep(MOVEMENT_BUFFER)
            for i in range(len(self.combined_intervals)):
                if self.stop_flag.is_set(): break
                self.root.after(0, lambda i=i: self.photo_count_var.set(f"Preview Step: {i+1} / {len(self.combined_intervals)}"))
                self._perform_motor_moves(i); time.sleep(0.05)
        except Exception as e: logging.error(f"Error in preview thread: {e}", exc_info=True)

    def _perform_motor_moves(self, interval_index):
        self.motors_moving = True
        slider_steps, rotation_steps = (0, 0)
        if interval_index < len(self.combined_intervals):
            interval_data = self.combined_intervals[interval_index]
            slider_steps, rotation_steps = interval_data.get('slider_steps', 0), interval_data.get('rotation_steps', 0)
            self.root.after(0, lambda s=slider_steps: self.live_slider_steps_var.set(str(s)))
            self.root.after(0, lambda r=rotation_steps: self.live_rotation_steps_var.set(str(r)))
            self.hardware.enable_motors()
            if slider_steps > 0: self.hardware.set_slider_step(slider_steps, self.direction_var.get() == "right")
            if self.stop_flag.is_set(): self.motors_moving = False; self.hardware.disable_motors(); return
            if rotation_steps != 0:
                is_forward = (self.hardware.rotation_direction_gpio == GPIO.HIGH) != (rotation_steps < 0)
                self.hardware.move_steps(ROTATION_DIR_PIN, ROTATION_STEP_PIN, abs(rotation_steps), is_forward)
            self.hardware.disable_motors()
        if interval_index >= len(self.combined_intervals) - 1:
            self.root.after(100, lambda: self.live_slider_steps_var.set("--")); self.root.after(100, lambda: self.live_rotation_steps_var.set("--"))
        self.motors_moving = False

    def _run_aux_trigger_listener_thread(self):
        logging.info("Auxiliary trigger listener thread started.")
        while not self.stop_flag.is_set():
            if self.gantry_running and self.holygrail_enabled_var.get() and GPIO.input(AUX_TRIGGER_PIN) == GPIO.LOW:
                if not self.aux_trigger_fired.is_set():
                    logging.info("Auxiliary trigger detected! Mirroring last XMP and capturing.")
                    if self.last_raw_filepath and os.path.exists(os.path.dirname(self.last_raw_filepath)):
                        base, ext = os.path.splitext(self.last_raw_filepath)
                        aux_raw_path = base + "_AUX" + ext
                        kelvin = self.holygrail_controller.kelvin_vals[-1] if self.holygrail_controller else 5200
                        self._create_xmp_sidecar(aux_raw_path, self.last_xmp_offset, kelvin)
                        self.camera.capture_and_download(aux_raw_path)
                        self.aux_trigger_fired.set()
                    else:
                        logging.warning("Aux trigger fired, but no previous shot data to mirror. Triggering simple shot.")
                        self.camera.trigger_photo()
                        self.aux_trigger_fired.set()
            time.sleep(0.05)
        logging.info("Auxiliary trigger listener thread stopped.")

    def monitor_operation_thread(self):
        if self.operation_thread and self.operation_thread.is_alive():
            self.root.after(100, self.monitor_operation_thread)
        else:
            self.gantry_running, self.motors_moving = False, False
            if self.root.winfo_exists():
                self.gui.update_ui_for_run_state(False); self.time_remaining_var.set("Time Rem: Finished")

    def on_closing(self):
        if self.gantry_running and messagebox.askyesno("Confirm Exit", "Operation in progress. Stop and exit?"):
            self.stop_flag.set()
            if self.operation_thread: self.operation_thread.join(timeout=5.0)
            if self.aux_trigger_thread: self.aux_trigger_thread.join(timeout=1.0)
            self.cleanup_and_exit_app()
        else:
            self.cleanup_and_exit_app()

    def cleanup_and_exit_app(self):
        self.stop_flag.set()
        if self.aux_trigger_thread and self.aux_trigger_thread.is_alive(): self.aux_trigger_thread.join(timeout=1.0)
        if self.camera: self.camera.cleanup()
        if self.root.winfo_exists(): self.root.destroy()
        logging.info("--- PiSlider Application Shutdown ---")

    def request_stop_operations(self):
        if self.gui.stop_button.winfo_exists(): self.gui.stop_button.config(state=tkinter.DISABLED)
        self.stop_flag.set()

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

    def manual_trigger_action(self):
        if self.mode_var.get() == 'manual' and self.gantry_running and not self.motors_moving: self.force_interval = True

    def _manual_move_wrapper(self, move_func, *args):
        if self.gantry_running or self.motors_moving: return
        threading.Thread(target=move_func, args=args, daemon=True).start()

    def rotate_cw_manual(self): self._manual_move_wrapper(self.hardware.rotate_motor, 1, True)
    def rotate_ccw_manual(self): self._manual_move_wrapper(self.hardware.rotate_motor, 1, False)
    def gantry_right_manual(self): self._manual_move_wrapper(self.hardware.set_slider_step, 50, True)
    def gantry_left_manual(self): self._manual_move_wrapper(self.hardware.set_slider_step, 50, False)
    def update_rotation_direction(self, event=None): self.hardware.rotation_direction_gpio = GPIO.HIGH if self.rotation_direction_var.get() == "CW" else GPIO.LOW
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
        if not self.camera.gphoto2_available: self.camera_status_var.set("Status: gphoto2 not installed."); return
        if self.camera_check_thread and self.camera_check_thread.is_alive(): return

        def _threaded_check():
            self.root.after(0, lambda: self.gui.check_camera_button.config(state=tkinter.DISABLED))
            self.root.after(0, lambda: self.camera_status_var.set("Status: Checking..."))
            summary = self.camera.get_camera_summary()
            self.root.after(0, lambda s=summary: self.camera_status_var.set(f"Status: {s}"))
            if self.root.winfo_exists():
                self.root.after(0, lambda: self.gui.camera_status_label.config(foreground='green' if "Connected" in summary else 'red'))
                self.root.after(0, lambda: self.gui.check_camera_button.config(state=tkinter.NORMAL))

        self.camera_check_thread = threading.Thread(target=_threaded_check, daemon=True); self.camera_check_thread.start()

    def on_camera_mode_change(self, event=None):
        self.camera.set_trigger_mode(self.camera_trigger_mode_var.get())
        is_usb = self.camera_trigger_mode_var.get() == "USB Control"
        usb_state = tkinter.NORMAL if is_usb and self.camera.gphoto2_available else tkinter.DISABLED
        if self.gui.take_preview_button: self.gui.take_preview_button.config(state=usb_state)
        if self.gui.check_camera_button: self.gui.check_camera_button.config(state=usb_state)
        if self.gui.calibrate_button: self.gui.calibrate_button.config(state=tkinter.NORMAL if is_usb and self.holygrail_enabled_var.get() else tkinter.DISABLED)
        self.camera_status_var.set("Status: S2 Cable Mode" if not is_usb else "Status: USB Mode (Unknown Connection)")
        if self.gui.camera_status_label: self.gui.camera_status_label.config(foreground='blue' if not is_usb else 'black')

    def calibrate_holygrail_exposure(self):
        if not self.camera.gphoto2_available or self.camera.trigger_mode != 'USB Control': messagebox.showwarning("Calibration Error", "Calibration requires USB Control mode."); return
        if self.operation_thread and self.operation_thread.is_alive(): messagebox.showwarning("Busy", "Another camera operation is in progress."); return

        def run_calib():
            self.root.after(0, lambda: self.gui.update_ui_for_run_state(True))
            self.root.after(0, lambda: self.calibration_offset_var.set("Reading settings..."))
            try:
                iso_str, aperture_str, shutter_str = self.camera.get_config_value("iso"), self.camera.get_config_value("f-number"), self.camera.get_config_value("shutterspeed")
                user_iso, user_aperture, user_shutter_s = int(iso_str), float(aperture_str.replace('f/', '')), self.camera.get_shutter_s(shutter_str)
            except Exception as e:
                logging.error(f"Could not read camera settings: {e}"); self.root.after(0, lambda: self.calibration_offset_var.set("Read Failed")); self.root.after(0, lambda: self.gui.update_ui_for_run_state(False)); return
            
            self.root.after(0, lambda: self.calibration_offset_var.set("Capturing..."))
            preview_dest = os.path.join(self.save_path_var.get(), "calibration.ARW")
            preview_for_ui = f"/tmp/ui_preview.jpg"
            if self.camera.capture_and_download(preview_dest, preview_for_ui):
                self.root.after(0, lambda p=preview_for_ui: self.gui.update_preview_image(p))
                
                temp_controller = HolyGrailController(
                    lat=float(self.holygrail_latitude_var.get()),
                    lon=float(self.holygrail_longitude_var.get()),
                    settings_table_dict=self.get_holygrail_settings_from_ui(),
                    day_interval_s=self.holygrail_day_interval_var.get(),
                    sunset_interval_s=self.holygrail_sunset_interval_var.get(),
                    night_interval_s=self.holygrail_night_interval_var.get(),
                    timezone_str=self.holygrail_timezone_var.get(),
                    iso_transition_duration_frames=self.holygrail_iso_transition_frames_var.get()
                )

                if temp_controller.calibrate(preview_for_ui, user_iso, user_aperture, user_shutter_s):
                    self.anchor_offset_ev = temp_controller.anchor_offset
                    self.root.after(0, lambda: self.calibration_offset_var.set(f"Anchor Offset: {self.anchor_offset_ev:+.2f}"))
            else: self.root.after(0, lambda: self.calibration_offset_var.set("Capture Failed"))
            self.root.after(0, lambda: self.gui.update_ui_for_run_state(False))
        self.preview_thread = threading.Thread(target=run_calib, daemon=True); self.preview_thread.start()

    def take_preview(self):
        if not self.camera.gphoto2_available or self.camera.trigger_mode != 'USB Control': messagebox.showwarning("Preview Error", "Preview requires USB Control mode."); return
        if self.preview_thread and self.preview_thread.is_alive(): messagebox.showwarning("Busy", "Another camera operation is in progress."); return

        def run_preview():
            self.root.after(0, lambda: self.gui.take_preview_button.config(state=tkinter.DISABLED))
            preview_dest = os.path.join(self.save_path_var.get(), "preview.ARW")
            preview_for_ui = f"/tmp/ui_preview.jpg"
            if self.camera.capture_and_download(preview_dest, preview_for_ui):
                self.root.after(0, lambda: self.gui.update_preview_image(preview_for_ui))
            else: self.root.after(0, lambda: messagebox.showerror("Preview Failed", "Could not capture the preview image."))
            if self.root.winfo_exists(): self.root.after(0, lambda: self.gui.take_preview_button.config(state=tkinter.NORMAL))
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
        except (ValueError, tkinter.TclError) as e: messagebox.showerror("Input Error", f"Invalid input value: {e}"); return False

    def calculate_distribution_arrays(self):
        try:
            num_photos = self.num_photos_var.get(); length_mm = int(self.length_var.get() or "0"); min_speed_mm = self.minimum_slider_speed_var.get()
            dist_pulse = (BELT_PITCH * PULLEY_TEETH) / (STEPS_PER_REVOLUTION_SLIDER * self.hardware.slider_microstepping) if self.hardware.slider_microstepping > 0 else 0
            self.total_steps = int(length_mm / dist_pulse) if dist_pulse > 0 else 0
            min_speed_pulses = int(min_speed_mm / dist_pulse) if dist_pulse > 0 else 0
            slider_func = self.distribution_functions.get(self.distribution_var.get(), self.even_distribution)
            if (min_speed_pulses * num_photos) >= self.total_steps and self.total_steps > 0:
                self.steps_distribution = self.scale_array_to_sum(slider_func(num_photos), self.total_steps)
            else:
                base_dist = self.scale_array_to_sum(slider_func(num_photos), self.total_steps - (min_speed_pulses * num_photos))
                self.steps_distribution = base_dist + min_speed_pulses
            if self.total_steps > 0 and self.steps_distribution.size > 0: self.steps_distribution[-1] += self.total_steps - np.sum(self.steps_distribution)
            rot_angle = self.rotation_angle_var.get()
            total_rot_pulses = int((rot_angle / 360.0) * STEPS_PER_REVOLUTION_ROTATION)
            rot_func = self.distribution_functions.get(self.rotation_distribution_var.get())
            rot_raw = rot_func(num_photos, rot_angle if self.rotation_distribution_var.get() == "object_tracking" else total_rot_pulses)
            rot_final = self.scale_array_to_sum(rot_raw, total_rot_pulses) if self.rotation_distribution_var.get() != "object_tracking" else rot_raw.astype(int)
            if total_rot_pulses != 0 and rot_final.size > 0: rot_final[-1] += total_rot_pulses - np.sum(rot_final)
            self.combined_intervals = [{'slider_steps': int(s), 'rotation_steps': int(r)} for s, r in zip(self.steps_distribution, rot_final)]
            return True
        except Exception as e: messagebox.showerror("Calculation Error", f"Failed to calculate distributions: {e}"); return False

    def scale_array_to_sum(self, arr_in, target_sum_val):
        arr = np.nan_to_num(np.array(arr_in, dtype=float)); target_sum = float(target_sum_val)
        if arr.size == 0: return np.array([], dtype=int)
        current_sum = np.sum(arr)
        if abs(current_sum) < 1e-9:
            if arr.size > 0 and target_sum != 0: base, rem = divmod(int(round(target_sum)), arr.size); return np.full(arr.size, base, dtype=int) + np.array([1]*rem + [0]*(arr.size-rem))
            return np.zeros(arr.size, dtype=int)
        scaled_float = arr * (target_sum / current_sum)
        int_arr = np.floor(scaled_float).astype(int)
        error_to_dist = int(round(target_sum)) - np.sum(int_arr)
        if error_to_dist > 0: indices = np.argsort(scaled_float - int_arr)[-error_to_dist:]; int_arr[indices] += 1
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
        self.live_ev_offset_var.set(f"{settings.get('reactive_ev_offset', 0):+.2f} EV")
        self.live_interval_var.set(f"{settings.get('target_interval', 0):.1f}s")

    def catenary_distribution(self, total_intervals, *args): return np.cosh(np.linspace(-1.5, 1.5, total_intervals)) - 1 if total_intervals > 0 else np.array([])
    def inverted_catenary_distribution(self, total_intervals, *args): s = self.catenary_distribution(total_intervals); return (np.max(s) if s.size > 0 else 0) - s
    def gaussian_distribution(self, total_intervals, *args): return np.exp(-np.linspace(-2.5, 2.5, total_intervals)**2 / 2) if total_intervals > 0 else np.array([])
    def inverted_gaussian_distribution(self, total_intervals, *args): s = self.gaussian_distribution(total_intervals); return (np.max(s) if s.size > 0 else 0) - s
    def ellipsoidal_distribution(self, total_intervals, *args): x = np.linspace(-1, 1, total_intervals); return np.sqrt(1 - x**2) if total_intervals > 0 else np.array([])
    def inverted_ellipsoidal_distribution(self, total_intervals, *args): s = self.ellipsoidal_distribution(total_intervals); return (np.max(s) if s.size > 0 else 0) - s
    def parabolic_distribution(self, total_intervals, *args): return 1 - np.linspace(-1, 1, total_intervals)**2 if total_intervals > 0 else np.array([])
    def inverted_parabolic_distribution(self, total_intervals, *args): s = self.parabolic_distribution(total_intervals); return (np.max(s) if s.size > 0 else 0) - s
    def cycloid_distribution(self, total_intervals, *args): return 1 - np.cos(np.linspace(0, 2 * np.pi, total_intervals)) if total_intervals > 0 else np.array([])
    def inverted_cycloid_distribution(self, total_intervals, *args): s = self.cycloid_distribution(total_intervals); return (np.max(s) if s.size > 0 else 0) - s
    def lame_curve_distribution(self, total_intervals, *args, n_param=4): x = np.linspace(-1, 1, total_intervals); return (1 - np.abs(x)**n_param)**(1/n_param) if total_intervals > 0 else np.array([])
    def inverted_lame_curve_distribution(self, total_intervals, *args): s = self.lame_curve_distribution(total_intervals); return (np.max(s) if s.size > 0 else 0) - s
    def linear_distribution(self, total_intervals, *args): return np.linspace(0.01, 1, total_intervals) if total_intervals > 0 else np.array([])
    def inverted_linear_distribution(self, total_intervals, *args): return np.linspace(1, 0.01, total_intervals) if total_intervals > 0 else np.array([])
    def even_distribution(self, total_intervals, *args): return np.ones(total_intervals) if total_intervals > 0 else np.array([])

    def object_tracking_rotation(self, total_intervals, total_rotation_angle_deg, *args):
        if total_intervals <= 0 or self.steps_distribution.size != total_intervals: return np.zeros(total_intervals)
        if self.total_steps == 0: return self.scale_array_to_sum(np.ones(total_intervals), int((total_rotation_angle_deg / 360.0) * STEPS_PER_REVOLUTION_ROTATION))
        dist_per_pulse = (BELT_PITCH * PULLEY_TEETH) / (STEPS_PER_REVOLUTION_SLIDER * self.hardware.slider_microstepping)
        slider_move_mm_interval, total_slider_travel_mm = self.steps_distribution * dist_per_pulse, self.total_steps * dist_per_pulse
        if abs(total_rotation_angle_deg) < 1e-3: return np.zeros(total_intervals)
        dist_to_obj_mm = (total_slider_travel_mm / 2.0) / math.tan(math.radians(abs(total_rotation_angle_deg)) / 2.0)
        cumulative_slider_pos_mm, initial_x_offset = 0.0, -total_slider_travel_mm / 2.0
        previous_angle_rad = math.atan2(initial_x_offset, dist_to_obj_mm)
        rotation_angles_rad_interval = np.zeros(total_intervals)
        for i in range(total_intervals):
            cumulative_slider_pos_mm += slider_move_mm_interval[i]
            current_x_offset = cumulative_slider_pos_mm - (total_slider_travel_mm / 2.0)
            target_angle_rad = math.atan2(current_x_offset, dist_to_obj_mm)
            rotation_angles_rad_interval[i], previous_angle_rad = target_angle_rad - previous_angle_rad, target_angle_rad
        return rotation_angles_rad_interval * (STEPS_PER_REVOLUTION_ROTATION / (2 * math.pi))

if __name__ == "__main__":
    root = tkinter.Tk()
    app = None
    try:
        try:
            root.tk.call('tk', 'scaling', 2.0)
        except tkinter.TclError:
            logging.warning("Could not set Tk scaling.")
        root.title("PiSlider Control")

        app = PiSliderApp(root)

        if root.winfo_exists():
            root.mainloop()

    except Exception as e:
        logging.critical(f"A fatal error occurred during application startup: {e}", exc_info=True)
        if root and root.winfo_exists():
            messagebox.showerror("Fatal Error", f"A critical error occurred: {e}\nApplication will exit.")
            root.destroy()
    finally:
        if app and hasattr(app, 'stop_flag'):
             app.stop_flag.set()
        logging.info("--- PiSlider Application Shutdown ---")
