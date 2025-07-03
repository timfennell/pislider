import threading
import time
import tkinter
from tkinter import ttk, messagebox, font, scrolledtext
import numpy as np
import RPi.GPIO as GPIO
import os
import math
import serial
import logging
from enum import Enum
import textwrap
import subprocess
import json
from holygrail import HolyGrailController
from PIL import Image, ImageTk

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s')

# --- Configuration Constants ---
IMAGE_DIR = "/home/tim/Desktop/PiSlider"
END_SWITCH_1_PIN = 10
END_SWITCH_2_PIN = 7
CAMERA_TRIGGER_PIN = 5
AUX_TRIGGER_PIN = 27
SLIDER_DIR_PIN = 26
SLIDER_STEP_PIN = 20
ROTATION_DIR_PIN = 2
ROTATION_STEP_PIN = 3
SLIDER_EN_PIN = 13
ROTATION_EN_PIN = 17
MOVEMENT_BUFFER = 2
STEPS_PER_REVOLUTION_SLIDER = 200 * 2
STEPS_PER_REVOLUTION_ROTATION = 48000
BELT_PITCH = 2
PULLEY_TEETH = 16
GPIO_HIGH = GPIO.HIGH
GPIO_LOW = GPIO.LOW
UART_PORT = '/dev/serial0'
UART_BAUDRATE = 115200

class MotorState(Enum):
    DISABLED = 0
    SLIDER_ENABLED = 1
    ROTATION_ENABLED = 2
    BOTH_ENABLED = 3

class MotorDirection(Enum):
    FORWARD = GPIO_HIGH
    REVERSE = GPIO_LOW

class CameraController:
    # This class is complete and correct
    def __init__(self, hardware_controller):
        self.hardware = hardware_controller
        self.trigger_mode = 'S2 Cable'
        self.gphoto2_available = self.check_gphoto2()
        logging.info(f"gphoto2 availability: {self.gphoto2_available}")
        if self.gphoto2_available:
            logging.info("USB control is available. Ensure your Sony camera is set to 'PC Remote' in the USB settings.")

    def check_gphoto2(self):
        try:
            subprocess.run(['gphoto2', '--version'], check=True, capture_output=True)
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            logging.warning("gphoto2 command not found or failed. USB control will be disabled.")
            return False

    def set_trigger_mode(self, mode):
        if mode == 'USB Control' and not self.gphoto2_available:
            logging.error("Cannot set mode to USB Control: gphoto2 is not available.")
            self.trigger_mode = 'S2 Cable'
        else:
            self.trigger_mode = mode
            logging.info(f"Camera trigger mode set to: {self.trigger_mode}")

    def trigger_photo(self):
        if self.trigger_mode == 'S2 Cable':
            return self.hardware.trigger_camera_s2()
        elif self.trigger_mode == 'USB Control':
            return self.trigger_camera_usb()
        else:
            logging.error(f"Unknown camera trigger mode: {self.trigger_mode}")
            return False

    def trigger_camera_usb(self):
        logging.info("Attempting to trigger camera via USB (gphoto2)...")
        try:
            result = subprocess.run(
                ['gphoto2', '--capture-image'],
                check=True, capture_output=True, text=True, timeout=15
            )
            logging.info("USB camera trigger successful.")
            return True
        except (FileNotFoundError, subprocess.TimeoutExpired, subprocess.CalledProcessError) as e:
            logging.error(f"gphoto2 trigger failed: {e}")
            if hasattr(e, 'stderr'): logging.error(f"gphoto2 stderr: {e.stderr}")
            return False
        except Exception as e:
            logging.error(f"An unexpected error occurred during USB trigger: {e}", exc_info=True)
            return False

    def get_camera_summary(self):
        if not self.gphoto2_available:
            return "Error: gphoto2 not installed."
        logging.info("Checking for connected USB camera...")
        try:
            result = subprocess.run(
                ['gphoto2', '--summary'], check=True, capture_output=True, text=True, timeout=10
            )
            for line in result.stdout.split('\n'):
                if 'Model:' in line:
                    model = line.split('Model:')[1].strip()
                    logging.info(f"Camera found: {model}")
                    return f"Connected: {model}"
            return "Error: Unknown camera detected."
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            logging.warning("gphoto2 --summary failed. No camera detected or camera is busy.")
            return "No Camera Found"
        except Exception as e:
            logging.error(f"An unexpected error during camera check: {e}", exc_info=True)
            return f"Error: {e}"

    def _set_config(self, config_path, value):
        if self.trigger_mode != 'USB Control': return False
        try:
            logging.debug(f"gphoto2: Setting {config_path} to {value}")
            subprocess.run(
                ['gphoto2', '--set-config-value', f"{config_path}={value}"],
                check=True, capture_output=True, text=True, timeout=5
            )
            return True
        except Exception as e:
            logging.error(f"Failed to set config '{config_path}' to '{value}': {e}")
            if hasattr(e, 'stderr'): logging.error(f"gphoto2 stderr: {e.stderr}")
            return False

    def set_aperture(self, f_number_value):
        return self._set_config("/main/capturesettings/f-number", f_number_value)

    def set_iso(self, iso_value):
        return self._set_config("/main/imgsettings/iso", iso_value)

    def set_shutter_speed(self, shutter_value):
        return self._set_config("/main/capturesettings/shutterspeed", shutter_value)

    def set_white_balance_kelvin(self, kelvin_value):
        if self._set_config("/main/imgsettings/whitebalance", "Choose Color Temperature"):
            return self._set_config("/main/imgsettings/colortemperature", kelvin_value)
        logging.warning("Could not set WB mode to 'Choose Color Temperature'. Temp not set.")
        return False
        
    def open_shutter(self):
        return self._set_config("/main/actions/bulb", 1)
        
    def close_shutter(self):
        return self._set_config("/main/actions/bulb", 0)

    def capture_preview_image(self, final_jpeg_path):
        if self.trigger_mode != 'USB Control':
            logging.warning("Cannot capture preview when not in USB Control mode.")
            return False
        
        temp_raw_path = "/tmp/pis_preview_temp.arw"
        
        os.makedirs(os.path.dirname(final_jpeg_path), exist_ok=True)
        
        logging.info("Capturing and downloading RAW thumbnail...")
        try:
            subprocess.run(
                ['gphoto2', '--capture-image-and-download', '--force-overwrite', f'--filename={temp_raw_path}'],
                check=True, capture_output=True, text=True, timeout=20
            )

            time.sleep(0.2)
            if not os.path.exists(temp_raw_path) or os.path.getsize(temp_raw_path) == 0:
                logging.error("gphoto2 command succeeded but failed to create a valid RAW file.")
                return False

            logging.info(f"Converting {temp_raw_path} to JPEG...")
            dcraw_command = f"dcraw -e -c {temp_raw_path} > {final_jpeg_path}"
            subprocess.run(dcraw_command, shell=True, check=True, capture_output=True, text=True, timeout=10)

            time.sleep(0.2)
            if os.path.exists(final_jpeg_path) and os.path.getsize(final_jpeg_path) > 0:
                logging.info(f"Preview image successfully converted and saved to {final_jpeg_path}")
                return True
            else:
                logging.error("dcraw conversion failed to produce a valid JPEG file.")
                return False

        except Exception as e:
            logging.error(f"Failed to capture or convert preview image: {e}", exc_info=True)
            if hasattr(e, 'stderr'): logging.error(f"gphoto2/dcraw stderr: {e.stderr}")
            return False
        finally:
            if os.path.exists(temp_raw_path):
                os.remove(temp_raw_path)

class HardwareController:
    # This entire class is unchanged
    def __init__(self):
        self.ser = None
        self.MICROSTEPPING = 4
        self.slider_microstepping = 4
        self.motor_state = MotorState.DISABLED

        self.slider_ihold_default, self.slider_irun_default, self.slider_iholddelay_default = 10, 25, 8
        self.rotation_ihold_default, self.rotation_irun_default, self.rotation_iholddelay_default = 5, 20, 8

        self.current_power_mode_is_low = False

        self.setup_gpio()
        self.stop_flag = threading.Event()

    def setup_gpio(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(END_SWITCH_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(END_SWITCH_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(AUX_TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            logging.info(f"AUX_TRIGGER_PIN ({AUX_TRIGGER_PIN}) configured as INPUT with PULL_UP.")
            GPIO.setup(CAMERA_TRIGGER_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(SLIDER_DIR_PIN, GPIO.OUT)
            GPIO.setup(SLIDER_STEP_PIN, GPIO.OUT)
            GPIO.setup(ROTATION_DIR_PIN, GPIO.OUT)
            GPIO.setup(ROTATION_STEP_PIN, GPIO.OUT)
            GPIO.setup(SLIDER_EN_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(ROTATION_EN_PIN, GPIO.OUT, initial=GPIO.HIGH)
        except Exception as e:
            logging.error(f"Error configuring GPIO: {e}", exc_info=True)
            raise

        try:
            self.ser = serial.Serial(UART_PORT, UART_BAUDRATE, rtscts=False, dsrdtr=False)
            logging.info(f"Serial port {UART_PORT} initialized at {UART_BAUDRATE} baud.")
        except serial.SerialException as e:
            logging.error(f"FATAL: Error opening serial port {UART_PORT}: {e}", exc_info=True)
            self.ser = None
            raise RuntimeError(f"Failed to open serial port {UART_PORT}.")
        self.configure_tmc2209_drivers()

    def build_tmc_write_command(self, register_address, value):
        command = bytearray([0x05, 0x00, register_address, 0x00])
        command.extend(value.to_bytes(4, 'big'))
        checksum = sum(command) % 256
        command.append(checksum)
        return bytes(command)

    def send_tmc_command(self, driver_type, command):
        if self.ser is None:
            logging.warning("Serial port not initialized, skipping TMC command.")
            return False

        if driver_type == "slider":
            GPIO.output(ROTATION_EN_PIN, GPIO.HIGH)
            GPIO.output(SLIDER_EN_PIN, GPIO.LOW)
        elif driver_type == "rotation":
            GPIO.output(SLIDER_EN_PIN, GPIO.HIGH)
            GPIO.output(ROTATION_EN_PIN, GPIO.LOW)
        else:
            logging.error(f"Invalid driver type for TMC command: {driver_type}")
            return False
        time.sleep(0.02)
        logging.debug(f"Sending UART to {driver_type}: {command.hex()}")
        success = False
        try:
            self.ser.write(command)
            time.sleep(0.02)
            success = True
        except Exception as e:
            logging.error(f"Serial write error for TMC command to {driver_type}: {e}", exc_info=True)
        return success


    def configure_tmc2209_drivers(self):
        if not self.ser:
            logging.error("Serial port not available. Cannot configure TMC drivers.")
            return

        slider_ihold_irun = (self.slider_ihold_default << 0) | \
                            (self.slider_irun_default << 8) | \
                            (self.slider_iholddelay_default << 16)
        rotation_ihold_irun = (self.rotation_ihold_default << 0) | \
                              (self.rotation_irun_default << 8) | \
                              (self.rotation_iholddelay_default << 16)

        gconf_for_stealth = 0x00000000
        gconf_for_spread  = 0x00000004

        logging.debug("Configuring Slider TMC2209 (StealthChop) with default currents.")
        self.send_tmc_command("slider", self.build_tmc_write_command(0x00, gconf_for_stealth))
        self.send_tmc_command("slider", self.build_tmc_write_command(0x10, slider_ihold_irun))
        self.send_tmc_command("slider", self.build_tmc_write_command(0x6C, 0x10000000 | (4 << 20) | (2 << 28)))
        self.send_tmc_command("slider", self.build_tmc_write_command(0x70, 0x00050403))
        self.set_microstepping("slider", self.slider_microstepping)

        logging.debug("Configuring Rotation TMC2209 (SpreadCycle) with default currents.")
        self.send_tmc_command("rotation", self.build_tmc_write_command(0x00, gconf_for_spread))
        self.send_tmc_command("rotation", self.build_tmc_write_command(0x10, rotation_ihold_irun))
        self.send_tmc_command("rotation", self.build_tmc_write_command(0x6C, 0x00010303 | (4 << 20) | (1 << 28)))
        self.set_microstepping("rotation", self.MICROSTEPPING)

        logging.info("TMC2209 drivers configured (default power).")
        GPIO.output(ROTATION_EN_PIN, GPIO.HIGH)
        GPIO.output(SLIDER_EN_PIN, GPIO.LOW)
        self.motor_state = MotorState.SLIDER_ENABLED
        self.current_power_mode_is_low = False

    def set_current_values(self, driver_type, ihold, irun, iholddelay):
        if not self.ser: return
        ihold_irun_val = (ihold << 0) | (irun << 8) | (iholddelay << 16)
        logging.info(f"Setting {driver_type} currents: IHOLD={ihold}, IRUN={irun}, DELAY={iholddelay} (0x{ihold_irun_val:08X})")
        self.send_tmc_command(driver_type, self.build_tmc_write_command(0x10, ihold_irun_val))

    def set_power_mode(self, low_power_active: bool):
        if low_power_active == self.current_power_mode_is_low:
            return

        MIN_CURRENT_SETTING = 1
        MAX_CURRENT_SETTING = 31

        if low_power_active:
            logging.info("Setting LOW POWER mode for motors (25% of full power).")

            slider_ihold_lp = max(MIN_CURRENT_SETTING, min(MAX_CURRENT_SETTING, int(round(self.slider_ihold_default * 0.25))))
            slider_irun_lp = max(MIN_CURRENT_SETTING, min(MAX_CURRENT_SETTING, int(round(self.slider_irun_default * 0.25))))
            slider_iholddelay_lp = self.slider_iholddelay_default
            self.set_current_values("slider", slider_ihold_lp, slider_irun_lp, slider_iholddelay_lp)

            rotation_ihold_lp = max(MIN_CURRENT_SETTING, min(MAX_CURRENT_SETTING, int(round(self.rotation_ihold_default * 0.25))))
            rotation_irun_lp = max(MIN_CURRENT_SETTING, min(MAX_CURRENT_SETTING, int(round(self.rotation_irun_default * 0.25))))
            rotation_iholddelay_lp = self.rotation_iholddelay_default
            self.set_current_values("rotation", rotation_ihold_lp, rotation_irun_lp, rotation_iholddelay_lp)
            
            self.current_power_mode_is_low = True
        else:
            logging.info("Setting FULL POWER mode for motors.")
            self.set_current_values("slider", self.slider_ihold_default, self.slider_irun_default, self.slider_iholddelay_default)
            self.set_current_values("rotation", self.rotation_ihold_default, self.rotation_irun_default, self.rotation_iholddelay_default)
            self.current_power_mode_is_low = False

        time.sleep(0.05)
        GPIO.output(ROTATION_EN_PIN, GPIO.HIGH)
        GPIO.output(SLIDER_EN_PIN, GPIO.LOW)
        self.motor_state = MotorState.SLIDER_ENABLED
        logging.info(f"Power mode set to {'low (25%)' if low_power_active else 'full'}. Motors reset to SLIDER_ENABLED state.")


    def set_microstepping(self, driver_type, microsteps=4):
        if microsteps not in [1, 2, 4, 8, 16, 32, 64, 128, 256]:
            logging.error(f"Invalid microstepping value: {microsteps}")
            return
        mres_val = 8 - int(math.log2(microsteps)) if microsteps > 0 else 0

        if driver_type == "slider":
            base_chopconf = 0x10000000 | (4 << 20) | (2 << 28)
            self.slider_microstepping = microsteps
        elif driver_type == "rotation":
            base_chopconf = 0x00010303 | (4 << 20) | (1 << 28)
            self.MICROSTEPPING = microsteps
        else:
            logging.error(f"Unknown driver type '{driver_type}' for set_microstepping.")
            return

        new_chopconf = (base_chopconf & ~(0xF << 24)) | (mres_val << 24)
        command = self.build_tmc_write_command(0x6C, new_chopconf)
        self.send_tmc_command(driver_type, command)
        logging.info(f"Microstepping for {driver_type} set to {microsteps}x (MRES val: {mres_val}). CHOPCONF: 0x{new_chopconf:08X}")

    def move_steps(self, enable_pin, dir_pin, step_pin, num_steps, forward=True):
        target_driver_type = "slider" if enable_pin == SLIDER_EN_PIN else "rotation"
        try:
            if target_driver_type == "slider":
                GPIO.output(ROTATION_EN_PIN, GPIO.HIGH)
                GPIO.output(SLIDER_EN_PIN, GPIO.LOW)
                self.motor_state = MotorState.SLIDER_ENABLED
            elif target_driver_type == "rotation":
                GPIO.output(SLIDER_EN_PIN, GPIO.LOW)
                GPIO.output(ROTATION_EN_PIN, GPIO.LOW)
                self.motor_state = MotorState.BOTH_ENABLED

            time.sleep(0.005)

            direction = GPIO_HIGH if forward else GPIO_LOW
            GPIO.output(dir_pin, direction)
            time.sleep(0.001)

            delay = 0.0005 if target_driver_type == "slider" else 0.00025

            for i in range(abs(num_steps)):
                if self.stop_flag.is_set():
                    logging.info(f"Stop flag detected during move_steps for {target_driver_type} after {i} steps.")
                    break
                GPIO.output(step_pin, GPIO.HIGH); time.sleep(delay)
                GPIO.output(step_pin, GPIO.LOW); time.sleep(delay)

            time.sleep(0.05)

            if target_driver_type == "slider": pass
            elif target_driver_type == "rotation":
                GPIO.output(ROTATION_EN_PIN, GPIO.HIGH)
                self.motor_state = MotorState.SLIDER_ENABLED

            logging.debug(f"Moved {target_driver_type} {num_steps} steps (dir:{'F'if forward else 'R'}). Motor state: {self.motor_state}")

        except Exception as e:
            logging.error(f"Error in move_steps for {target_driver_type} (en_pin {enable_pin}): {e}", exc_info=True)
            try:
                GPIO.output(ROTATION_EN_PIN, GPIO.HIGH)
                GPIO.output(SLIDER_EN_PIN, GPIO.LOW)
                self.motor_state = MotorState.SLIDER_ENABLED
                logging.info("Reverted motor state to SLIDER_ENABLED after error in move_steps.")
            except Exception as e_reset:
                logging.error(f"Failed to reset motor state after error in move_steps: {e_reset}")

    def set_slider_step(self, num_steps, direction_is_high):
        if self.end_switch_triggered_by_hw():
            logging.warning("End switch triggered (checked by HW controller). Halting slider movement.")
            return
        self.move_steps(SLIDER_EN_PIN, SLIDER_DIR_PIN, SLIDER_STEP_PIN, num_steps, direction_is_high)

    def rotate_motor(self, angle_change_deg, rotation_direction_is_high):
        if angle_change_deg == 0: logging.debug("rotate_motor called with angle 0."); return
        rotation_steps = int((angle_change_deg / 360.0) * STEPS_PER_REVOLUTION_ROTATION)
        logging.info(f"rotate_motor: Angle {angle_change_deg} deg, Steps {rotation_steps}, DirHigh: {rotation_direction_is_high}")
        if rotation_steps != 0 :
             self.move_steps(ROTATION_EN_PIN, ROTATION_DIR_PIN, ROTATION_STEP_PIN, rotation_steps, rotation_direction_is_high)

    def trigger_camera_s2(self):
        try:
            logging.info(f"Attempting to trigger camera via S2 Cable (pin {CAMERA_TRIGGER_PIN}).")
            GPIO.output(CAMERA_TRIGGER_PIN, GPIO.HIGH)
            time.sleep(0.2)
            GPIO.output(CAMERA_TRIGGER_PIN, GPIO.LOW)
            logging.info("S2 camera trigger sequence completed.")
            return True
        except Exception as e:
            logging.error(f"Error triggering S2 camera: {e}", exc_info=True)
            return False

    def end_switch_triggered_by_hw(self):
        try:
            return GPIO.input(END_SWITCH_1_PIN) == GPIO.LOW or GPIO.input(END_SWITCH_2_PIN) == GPIO.LOW
        except Exception as e:
            logging.error(f"Error reading end switches in HW controller: {e}", exc_info=True)
            return False

    def move_to_end(self, direction_pin_state_is_high_for_this_move):
        end_name_suffix = " (DIR=HIGH)" if direction_pin_state_is_high_for_this_move else " (DIR=LOW)"
        logging.info(f"Moving to an end switch for homing/positioning{end_name_suffix}...")

        GPIO.output(ROTATION_EN_PIN, GPIO.HIGH)
        GPIO.output(SLIDER_EN_PIN, GPIO.LOW)
        self.motor_state = MotorState.SLIDER_ENABLED
        time.sleep(0.01)

        GPIO.output(SLIDER_DIR_PIN, GPIO_HIGH if direction_pin_state_is_high_for_this_move else GPIO_LOW)
        time.sleep(0.001)

        slider_delay = 0.0005
        max_init_steps, steps_taken = 300000, 0

        while not self.end_switch_triggered_by_hw():
            if self.stop_flag.is_set():
                logging.info("Stop flag detected during move_to_end's stepping loop.")
                break
            if steps_taken >= max_init_steps:
                logging.error(f"move_to_end: Exceeded max steps ({max_init_steps}).")
                break
            GPIO.output(SLIDER_STEP_PIN, GPIO.HIGH); time.sleep(slider_delay)
            GPIO.output(SLIDER_STEP_PIN, GPIO.LOW); time.sleep(slider_delay)
            steps_taken += 1
            if steps_taken % 5000 == 0: logging.debug(f"move_to_end: {steps_taken} steps...")

        if self.end_switch_triggered_by_hw() and not self.stop_flag.is_set():
            logging.info("End switch triggered during move_to_end.")
            away_dir_pin_state_is_high = not direction_pin_state_is_high_for_this_move
            dist_per_pulse = (BELT_PITCH*PULLEY_TEETH)/(STEPS_PER_REVOLUTION_SLIDER*self.slider_microstepping) if self.slider_microstepping>0 else 0
            away_steps = int(2 / dist_per_pulse) if dist_per_pulse > 0 else 200
            if away_steps <=0 : away_steps = 200
            logging.info(f"Moving {away_steps} steps away from end switch.")
            self.move_steps(SLIDER_EN_PIN, SLIDER_DIR_PIN, SLIDER_STEP_PIN, away_steps, away_dir_pin_state_is_high)
            logging.info(f"Gantry position after moving away from switch.")
        elif self.stop_flag.is_set(): logging.warning("move_to_end: Stopped by user (stop_flag was set).")
        else: logging.warning("move_to_end: Max steps reached without triggering end switch.")

        GPIO.output(ROTATION_EN_PIN, GPIO.HIGH)
        GPIO.output(SLIDER_EN_PIN, GPIO.LOW)
        self.motor_state = MotorState.SLIDER_ENABLED

    def cleanup(self):
        logging.info("Cleaning up HardwareController resources...")
        try:
            if self.ser and self.ser.is_open:
                logging.info("Cleanup: Setting full power mode.")
                self.set_power_mode(low_power_active=False)
        except Exception as e:
            logging.error(f"Error setting full power during cleanup: {e}")

        try:
            if GPIO.getmode() is not None:
                if 'SLIDER_EN_PIN' in globals() and GPIO.gpio_function(SLIDER_EN_PIN) == GPIO.OUT: GPIO.output(SLIDER_EN_PIN, GPIO.HIGH)
                if 'ROTATION_EN_PIN' in globals() and GPIO.gpio_function(ROTATION_EN_PIN) == GPIO.OUT: GPIO.output(ROTATION_EN_PIN, GPIO.HIGH)
                self.motor_state = MotorState.DISABLED; time.sleep(0.1)
                GPIO.cleanup(); logging.info("GPIO cleanup successful.")
            else: logging.info("GPIO mode not set, skipping GPIO.cleanup().")
        except Exception as e: logging.error(f"Error during GPIO cleanup: {e}", exc_info=True)

        if self.ser and self.ser.is_open:
            try: self.ser.close(); logging.info("Serial port closed.")
            except Exception as e: logging.error(f"Error closing serial port: {e}", exc_info=True)
        self.ser = None

    def set_stop_flag_reference(self, stop_flag_event): self.stop_flag = stop_flag_event


class PiSliderApp:
    def __init__(self, root):
        self.root = root
        try: self.root.tk.call('tk', 'scaling', 2.0)
        except tkinter.TclError: logging.warning("Could not set Tk scaling.")
        self.root.title("PiSlider Camera Control")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.stop_flag = threading.Event()

        try:
            self.hardware = HardwareController()
            self.hardware.set_stop_flag_reference(self.stop_flag)
            self.camera = CameraController(self.hardware)
        except RuntimeError as e:
            logging.critical(f"HardwareController initialization failed: {e}", exc_info=True)
            messagebox.showerror("Hardware Error", f"Failed to initialize hardware: {e}\nApplication cannot continue.")
            self.hardware = None; self.camera = None
            if self.root.winfo_exists(): self.root.destroy()
            return
        except Exception as e:
            logging.critical(f"Unexpected error during HardwareController init: {e}", exc_info=True)
            messagebox.showerror("Critical Error", f"An unexpected error occurred: {e}\nApplication will exit.")
            self.hardware = None; self.camera = None
            if self.root.winfo_exists(): self.root.destroy()
            return

        # --- Initialize UI component references ---
        self.notebook = None
        self.control_panel_frame = None
        self.preview_frame = None
        
        self.preview_image_path = "/tmp/pis_live_preview.jpg"
        self.preview_thread = None
        self.preview_image_label = None
        self.take_preview_button = None
        self.preview_photo_image = None

        self.gantry_running = False; self.photos_taken = 0
        self.user_direction_ui = tkinter.StringVar(value='right')
        self.rotation_direction_gpio = GPIO_HIGH
        self.total_steps = 0; self.manual_mode = False
        self.force_interval = False; self.motors_moving = False
        self.aux_trigger_active = False; self.last_aux_pin_state = -1
        self.skip_next_scheduled_shot_due_to_aux = False

        self.direction_var = self.user_direction_ui
        self.length_var = tkinter.StringVar(value="2800")
        self.distribution_var = tkinter.StringVar(value="even")
        self.rotation_angle_var = tkinter.DoubleVar(value=90.0)
        self.rotation_direction_var = tkinter.StringVar(value="CW")
        self.rotation_distribution_var = tkinter.StringVar(value="even")
        self.num_photos_var = tkinter.IntVar(value=10)
        self.interval_var = tkinter.DoubleVar(value=5.0)
        self.exposure_pause_var = tkinter.DoubleVar(value=1.0)
        self.minimum_slider_speed_var = tkinter.IntVar(value=0)
        self.post_timelapse_position_var = tkinter.StringVar(value="Do Nothing")
        self.low_power_mode_var = tkinter.BooleanVar(value=False)
        self.camera_trigger_mode_var = tkinter.StringVar(value="S2 Cable")
        self.camera_status_var = tkinter.StringVar(value="Status: Unknown")
        
        self.holygrail_enabled_var = tkinter.BooleanVar(value=False)
        self.holygrail_latitude_var = tkinter.StringVar(value="48.177577")
        self.holygrail_longitude_var = tkinter.StringVar(value="-82.53815")
        self.holygrail_min_aperture_var = tkinter.DoubleVar(value=2.8)
        self.holygrail_max_aperture_var = tkinter.DoubleVar(value=8.0)
        self.holygrail_base_iso_var = tkinter.IntVar(value=100)
        self.holygrail_max_iso_var = tkinter.IntVar(value=6400)
        
        self.camera_check_thread = None
        self.camera_check_result = None
        self.distribution_photo_image = None

        self.distribution_images = {
            "catenary": "catenary.png", "inverted_catenary": "inverted_catenary.png",
            "gaussian": "gaussian.png", "inverted_gaussian": "inverted_gaussian.png",
            "ellipsoidal": "ellipsoidal.png", "inverted_ellipsoidal": "inverted_ellipsoidal.png",
            "parabolic": "parabolic.png", "inverted_parabolic": "inverted_parabolic.png",
            "cycloid": "cycloid.png", "inverted_cycloid": "inverted_cycloid.png",
            "lame_curve": "lame_curve.png", "inverted_lame_curve": "inverted_lame_curve.png",
            "linear": "linear.png", "inverted_linear": "inverted_linear.png",
            "even": "even.png"
        }
        
        self.distribution_functions = { "catenary": self.catenary_distribution, "inverted_catenary": self.inverted_catenary_distribution, "gaussian": self.gaussian_distribution, "inverted_gaussian": self.inverted_gaussian_distribution, "ellipsoidal": self.ellipsoidal_distribution, "inverted_ellipsoidal": self.inverted_ellipsoidal_distribution, "parabolic": self.parabolic_distribution, "inverted_parabolic": self.inverted_parabolic_distribution, "cycloid": self.cycloid_distribution, "inverted_cycloid": self.inverted_cycloid_distribution, "lame_curve": self.lame_curve_distribution, "inverted_lame_curve": self.inverted_lame_curve_distribution, "linear": self.linear_distribution, "inverted_linear": self.inverted_linear_distribution, "even": self.even_distribution, "object_tracking": self.object_tracking_rotation }
        
        self.create_widgets(); self.bind_keys()
        self.update_rotation_direction(); self.update_distribution_image()
        self.on_camera_mode_change()
        self.toggle_holygrail_mode()
        self.entry_has_focus = False

    def create_widgets(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(side=tkinter.LEFT, fill=tkinter.BOTH, expand=True, padx=5, pady=5)
        
        # --- Store reference to the preview frame ---
        self.preview_frame = ttk.Frame(self.root, padding=5, relief="groove", borderwidth=2)
        self.preview_frame.pack(side=tkinter.RIGHT, fill=tkinter.BOTH, expand=True, padx=5, pady=5)
        
        paned_window = ttk.PanedWindow(main_frame, orient=tkinter.HORIZONTAL)
        paned_window.pack(fill=tkinter.BOTH, expand=True)

        settings_pane = ttk.Frame(paned_window, padding="10")
        
        # --- Store reference to the control panel frame ---
        self.control_panel_frame = ttk.Frame(paned_window, padding="10")
        
        paned_window.add(settings_pane, weight=2)
        paned_window.add(self.control_panel_frame, weight=1)

        # --- Store reference to the notebook ---
        self.notebook = ttk.Notebook(settings_pane)
        self.notebook.pack(fill=tkinter.BOTH, expand=True)

        main_settings_frame = ttk.Frame(self.notebook, padding="10")
        holygrail_frame = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(main_settings_frame, text='Main Settings')
        self.notebook.add(holygrail_frame, text='Holy Grail')

        bold_font = ('Arial', 12, 'bold');

        self._create_main_settings_widgets(main_settings_frame, bold_font)
        self._create_holygrail_widgets(holygrail_frame, bold_font)
        self._create_control_panel_widgets(self.control_panel_frame, bold_font)
        self._create_preview_widgets(self.preview_frame, bold_font)

        self.update_mode()

    def update_ui_for_run_state(self, is_running):
        """ Robustly updates the UI state by using direct widget references. """
        state = tkinter.DISABLED if is_running else tkinter.NORMAL
        try:
            # Disable notebook tabs using the stored reference
            if self.notebook:
                for i in range(len(self.notebook.tabs())):
                    self.notebook.tab(i, state=state)
            
            # Disable control panel widgets using the stored reference
            if self.control_panel_frame:
                for widget in self.control_panel_frame.winfo_children():
                    if isinstance(widget, (ttk.Button, ttk.Radiobutton, ttk.Entry, ttk.Combobox, ttk.Checkbutton)):
                        # Don't disable the Stop button when a run starts
                        if widget != self.stop_button:
                            widget.config(state=state)

            # Disable preview panel widgets using the stored reference
            if self.preview_frame:
                 for widget in self.preview_frame.winfo_children():
                    if isinstance(widget, ttk.Button):
                        widget.config(state=state)

            # Explicitly set the Stop button state
            if self.stop_button:
                self.stop_button.config(state=tkinter.NORMAL if is_running else tkinter.DISABLED)

            if not is_running:
                # Restore UI state based on current selections when stopping
                self.update_mode()
                self.on_camera_mode_change()
                self.toggle_holygrail_mode()

        except Exception as e:
            logging.error(f"Error updating UI state: {e}", exc_info=True)

    def _create_preview_widgets(self, parent_frame, bold_font):
        parent_frame.columnconfigure(0, weight=1)
        self.take_preview_button = ttk.Button(parent_frame, text="Take Preview", command=self.take_preview)
        self.take_preview_button.grid(row=0, column=0, pady=(5, 5))
        self.preview_image_label = ttk.Label(parent_frame, text="Preview will be shown here", anchor="center")
        self.preview_image_label.grid(row=1, column=0, sticky="nsew")
        parent_frame.rowconfigure(1, weight=1)

    def take_preview(self):
        if self.preview_thread and self.preview_thread.is_alive():
            logging.warning("Preview capture already in progress.")
            return
        if self.camera_trigger_mode_var.get() != "USB Control":
            messagebox.showinfo("Mode Error", "Preview capture is only available in USB Control mode.")
            return
        self.take_preview_button.config(state=tkinter.DISABLED)
        self.preview_image_label.config(text="Capturing...")
        self.preview_thread = threading.Thread(target=self._threaded_preview_capture, daemon=True)
        self.preview_thread.start()
        self.root.after(100, self._update_preview_image)

    def _threaded_preview_capture(self):
        if self.camera:
            self.camera.capture_preview_image(self.preview_image_path)

    def _update_preview_image(self):
        if self.preview_thread and self.preview_thread.is_alive():
            self.root.after(100, self._update_preview_image)
            return
        if self.take_preview_button: self.take_preview_button.config(state=tkinter.NORMAL)
        if not os.path.exists(self.preview_image_path):
            self.preview_image_label.config(text="Preview capture failed.")
            return
        try:
            img = Image.open(self.preview_image_path)
            target_w = self.preview_frame.winfo_width()
            target_h = self.preview_frame.winfo_height()
            if target_w < 10 or target_h < 10:
                target_w, target_h = 400, 300
            img.thumbnail((target_w - 10, target_h - 40), Image.Resampling.LANCZOS)
            self.preview_photo_image = ImageTk.PhotoImage(img)
            self.preview_image_label.config(image=self.preview_photo_image, text="")
        except Exception as e:
            logging.error(f"Error displaying preview image: {e}", exc_info=True)
            self.preview_image_label.config(image='', text="Error displaying preview.")
            
    def apply_power_mode_if_idle(self):
        if not self.gantry_running and self.hardware:
            is_low_power = self.low_power_mode_var.get()
            logging.info(f"Low power mode toggled to: {is_low_power} (while idle). Applying.")
            self.hardware.set_power_mode(low_power_active=is_low_power)
        elif self.gantry_running:
            logging.info("Low power mode changed, will be applied at the start of the next run.")

    def _create_main_settings_widgets(self, parent_frame, bold_font):
        parent_frame.columnconfigure(1, weight=1)
        current_row = 0
        ttk.Label(parent_frame, text="Slider Settings", font=bold_font).grid(row=current_row, column=0, columnspan=2, pady=5, sticky=tkinter.W); current_row += 1
        ttk.Label(parent_frame, text="Direction:").grid(row=current_row, column=0, sticky=tkinter.W)
        self.direction_combo = ttk.Combobox(parent_frame, textvariable=self.direction_var, values=['right', 'left'], width=18, state="readonly")
        self.direction_combo.grid(row=current_row, column=1, sticky=tkinter.E); current_row += 1
        ttk.Label(parent_frame, text="Length (mm):").grid(row=current_row, column=0, sticky=tkinter.W)
        self.length_entry = ttk.Entry(parent_frame, textvariable=self.length_var, width=20)
        self.length_entry.grid(row=current_row, column=1, sticky=tkinter.E)
        self.length_entry.bind("<FocusIn>", self.on_entry_focus)
        self.length_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Slider Distribution:").grid(row=current_row, column=0, sticky=tkinter.W)
        slider_dist_opts = list(self.distribution_images.keys())
        self.distribution_combo = ttk.Combobox(parent_frame, textvariable=self.distribution_var, values=slider_dist_opts, width=18, state="readonly")
        self.distribution_combo.grid(row=current_row, column=1, sticky=tkinter.E)
        self.distribution_combo.bind("<<ComboboxSelected>>", self.update_distribution_image); current_row += 1
        ttk.Label(parent_frame, text="Rotation Settings", font=bold_font).grid(row=current_row, column=0, columnspan=2, pady=(10,5), sticky=tkinter.W); current_row += 1
        ttk.Label(parent_frame, text="Rotation Angle (°):").grid(row=current_row, column=0, sticky=tkinter.W)
        self.rotation_angle_entry = ttk.Entry(parent_frame, textvariable=self.rotation_angle_var, width=20)
        self.rotation_angle_entry.grid(row=current_row, column=1, sticky=tkinter.E)
        self.rotation_angle_entry.bind("<FocusIn>", self.on_entry_focus)
        self.rotation_angle_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Rotation Direction:").grid(row=current_row, column=0, sticky=tkinter.W)
        self.rotation_direction_combo = ttk.Combobox(parent_frame, textvariable=self.rotation_direction_var, values=["CW", "CCW"], width=18, state="readonly")
        self.rotation_direction_combo.grid(row=current_row, column=1, sticky=tkinter.E)
        self.rotation_direction_combo.bind("<<ComboboxSelected>>", self.update_rotation_direction); current_row += 1
        ttk.Label(parent_frame, text="Rotation Distribution:").grid(row=current_row, column=0, sticky=tkinter.W)
        rot_dist_opts = list(self.distribution_functions.keys())
        self.rotation_distribution_combo = ttk.Combobox(parent_frame, textvariable=self.rotation_distribution_var, values=rot_dist_opts, width=18, state="readonly")
        self.rotation_distribution_combo.grid(row=current_row, column=1, sticky=tkinter.E); current_row += 1
        ttk.Label(parent_frame, text="General Settings", font=bold_font).grid(row=current_row, column=0, columnspan=2, pady=(10,5), sticky=tkinter.W); current_row += 1
        ttk.Label(parent_frame, text="Camera Trigger:").grid(row=current_row, column=0, sticky=tkinter.W)
        camera_modes = ["S2 Cable"]
        if self.camera and self.camera.gphoto2_available: camera_modes.append("USB Control")
        self.camera_trigger_combo = ttk.Combobox(parent_frame, textvariable=self.camera_trigger_mode_var, values=camera_modes, width=18, state="readonly")
        self.camera_trigger_combo.grid(row=current_row, column=1, sticky=tkinter.E)
        self.camera_trigger_combo.bind("<<ComboboxSelected>>", self.on_camera_mode_change); current_row += 1
        self.camera_status_label = ttk.Label(parent_frame, textvariable=self.camera_status_var)
        self.camera_status_label.grid(row=current_row, column=0, sticky=tkinter.W, columnspan=1)
        self.check_camera_button = ttk.Button(parent_frame, text="Check USB", command=self.check_camera_connection)
        self.check_camera_button.grid(row=current_row, column=1, sticky=tkinter.E); current_row += 1
        ttk.Label(parent_frame, text="Number of Photos:").grid(row=current_row, column=0, sticky=tkinter.W)
        self.num_photos_entry = ttk.Entry(parent_frame, textvariable=self.num_photos_var, width=20)
        self.num_photos_entry.grid(row=current_row, column=1, sticky=tkinter.E)
        self.num_photos_entry.bind("<FocusIn>", self.on_entry_focus)
        self.num_photos_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Interval (s):").grid(row=current_row, column=0, sticky=tkinter.W)
        self.interval_entry = ttk.Entry(parent_frame, textvariable=self.interval_var, width=20)
        self.interval_entry.grid(row=current_row, column=1, sticky=tkinter.E)
        self.interval_entry.bind("<FocusIn>", self.on_entry_focus)
        self.interval_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Exposure Pause (s):").grid(row=current_row, column=0, sticky=tkinter.W)
        self.exposure_pause_entry = ttk.Entry(parent_frame, textvariable=self.exposure_pause_var, width=20)
        self.exposure_pause_entry.grid(row=current_row, column=1, sticky=tkinter.E)
        self.exposure_pause_entry.bind("<FocusIn>", self.on_entry_focus)
        self.exposure_pause_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        for child in parent_frame.winfo_children(): child.grid_configure(padx=5, pady=2)

    def _create_holygrail_widgets(self, parent_frame, bold_font):
        parent_frame.columnconfigure(1, weight=1)
        current_row = 0
        ttk.Checkbutton(parent_frame, text="Enable Holy Grail Mode", variable=self.holygrail_enabled_var, command=self.toggle_holygrail_mode).grid(row=current_row, column=0, columnspan=2, sticky='w', pady=10)
        current_row += 1
        ttk.Label(parent_frame, text="Latitude:").grid(row=current_row, column=0, sticky='w')
        lat_entry = ttk.Entry(parent_frame, textvariable=self.holygrail_latitude_var)
        lat_entry.grid(row=current_row, column=1, sticky='ew')
        lat_entry.bind("<FocusIn>", self.on_entry_focus)
        lat_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Longitude:").grid(row=current_row, column=0, sticky='w')
        lon_entry = ttk.Entry(parent_frame, textvariable=self.holygrail_longitude_var)
        lon_entry.grid(row=current_row, column=1, sticky='ew')
        lon_entry.bind("<FocusIn>", self.on_entry_focus)
        lon_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Min Aperture (f-number):").grid(row=current_row, column=0, sticky='w')
        min_aperture_entry = ttk.Entry(parent_frame, textvariable=self.holygrail_min_aperture_var)
        min_aperture_entry.grid(row=current_row, column=1, sticky='ew')
        min_aperture_entry.bind("<FocusIn>", self.on_entry_focus)
        min_aperture_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Max Aperture (f-number):").grid(row=current_row, column=0, sticky='w')
        max_aperture_entry = ttk.Entry(parent_frame, textvariable=self.holygrail_max_aperture_var)
        max_aperture_entry.grid(row=current_row, column=1, sticky='ew')
        max_aperture_entry.bind("<FocusIn>", self.on_entry_focus)
        max_aperture_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Base ISO:").grid(row=current_row, column=0, sticky='w')
        base_iso_entry = ttk.Entry(parent_frame, textvariable=self.holygrail_base_iso_var)
        base_iso_entry.grid(row=current_row, column=1, sticky='ew')
        base_iso_entry.bind("<FocusIn>", self.on_entry_focus)
        base_iso_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Max ISO:").grid(row=current_row, column=0, sticky='w')
        max_iso_entry = ttk.Entry(parent_frame, textvariable=self.holygrail_max_iso_var)
        max_iso_entry.grid(row=current_row, column=1, sticky='ew')
        max_iso_entry.bind("<FocusIn>", self.on_entry_focus)
        max_iso_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row += 1
        ttk.Label(parent_frame, text="Settings Table (JSON):", font=bold_font).grid(row=current_row, column=0, columnspan=2, sticky='w', pady=(10,0)); current_row += 1
        self.holygrail_settings_text = scrolledtext.ScrolledText(parent_frame, height=8, width=40, wrap=tkinter.WORD)
        self.holygrail_settings_text.grid(row=current_row, column=0, columnspan=2, sticky='ew', pady=5)
        self.holygrail_settings_text.bind("<FocusIn>", self.on_entry_focus)
        self.holygrail_settings_text.bind("<FocusOut>", self.on_entry_defocus)
        default_settings_str = json.dumps(HolyGrailController.DEFAULT_SETTINGS_TABLE, indent=4)
        self.holygrail_settings_text.insert(tkinter.INSERT, default_settings_str)
        for child in parent_frame.winfo_children(): child.grid_configure(padx=5, pady=2)

    def _create_control_panel_widgets(self, parent_frame, bold_font):
        parent_frame.columnconfigure(1, weight=1)
        current_row_ctrl = 0
        ttk.Label(parent_frame, text="Control Panel", font=bold_font).grid(row=current_row_ctrl, column=0, columnspan=2, pady=5); current_row_ctrl += 1
        self.start_button = ttk.Button(parent_frame, text="Start Timelapse", command=self.start_timelapse)
        self.start_button.grid(row=current_row_ctrl, column=0, columnspan=2, pady=5, sticky="ew"); current_row_ctrl += 1
        self.preview_button = ttk.Button(parent_frame, text="Run Preview", command=self.start_preview)
        self.preview_button.grid(row=current_row_ctrl, column=0, columnspan=2, pady=5, sticky="ew"); current_row_ctrl += 1
        self.stop_button = ttk.Button(parent_frame, text="Stop", command=self.request_stop_operations, state=tkinter.DISABLED)
        self.stop_button.grid(row=current_row_ctrl, column=0, columnspan=2, pady=5, sticky="ew"); current_row_ctrl += 1
        self.photo_count_label = ttk.Label(parent_frame, text="Photos: 0 / 0")
        self.photo_count_label.grid(row=current_row_ctrl, column=0, columnspan=2, pady=5); current_row_ctrl += 1
        self.time_remaining_label = ttk.Label(parent_frame, text="Time Rem: --:--")
        self.time_remaining_label.grid(row=current_row_ctrl, column=0, columnspan=2, pady=5); current_row_ctrl += 1
        self.mode_var = tkinter.StringVar(value='timed')
        ttk.Label(parent_frame, text="Mode Selection", font=bold_font).grid(row=current_row_ctrl, column=0, columnspan=2, pady=5); current_row_ctrl += 1
        self.timed_radio = ttk.Radiobutton(parent_frame, text="Timed", variable=self.mode_var, value="timed", command=self.update_mode)
        self.timed_radio.grid(row=current_row_ctrl, column=0, sticky=tkinter.W)
        self.manual_radio = ttk.Radiobutton(parent_frame, text="Manual Trigger", variable=self.mode_var, value="manual", command=self.update_mode)
        self.manual_radio.grid(row=current_row_ctrl, column=1, sticky=tkinter.W); current_row_ctrl += 1
        ttk.Label(parent_frame, text="Min Slider Speed (mm/photo):").grid(row=current_row_ctrl, column=0, sticky=tkinter.W)
        self.minimum_slider_speed_entry = ttk.Entry(parent_frame, textvariable=self.minimum_slider_speed_var, width=20)
        self.minimum_slider_speed_entry.grid(row=current_row_ctrl, column=1, sticky=tkinter.E)
        self.minimum_slider_speed_entry.bind("<FocusIn>", self.on_entry_focus)
        self.minimum_slider_speed_entry.bind("<FocusOut>", self.on_entry_defocus)
        current_row_ctrl += 1
        ttk.Label(parent_frame, text="Power Mode:", font=bold_font).grid(row=current_row_ctrl, column=0, columnspan=2, pady=(10,0), sticky=tkinter.W); current_row_ctrl += 1
        self.low_power_checkbutton = ttk.Checkbutton(parent_frame, text="Low Power (Horizontal Only)", variable=self.low_power_mode_var, command=self.apply_power_mode_if_idle)
        self.low_power_checkbutton.grid(row=current_row_ctrl, column=0, columnspan=2, sticky=tkinter.W, pady=(0,5)); current_row_ctrl +=1
        ttk.Label(parent_frame, text="Post-Timelapse Action", font=bold_font).grid(row=current_row_ctrl, column=0, columnspan=2, pady=(5,5), sticky=tkinter.W); current_row_ctrl += 1
        ttk.Label(parent_frame, text="Gantry Position:").grid(row=current_row_ctrl, column=0, sticky=tkinter.W)
        self.post_timelapse_combo = ttk.Combobox(parent_frame, textvariable=self.post_timelapse_position_var, values=["Do Nothing", "Return to Start", "Move to End"], width=18, state="readonly")
        self.post_timelapse_combo.grid(row=current_row_ctrl, column=1, sticky=tkinter.E); current_row_ctrl += 1
        self.distribution_image_label = ttk.Label(parent_frame, anchor="center")
        self.distribution_image_label.grid(row=current_row_ctrl, column=0, columnspan=2, pady=10, sticky="ew")
        current_row_ctrl += 1
        for child in parent_frame.winfo_children(): child.grid_configure(padx=5, pady=2)
            
    def check_camera_connection(self):
        if self.camera_check_thread and self.camera_check_thread.is_alive():
            logging.warning("Camera check is already in progress.")
            return
        self.check_camera_button.config(state=tkinter.DISABLED)
        self.camera_status_var.set("Status: Checking...")
        self.camera_status_label.config(foreground='black')
        self.camera_check_thread = threading.Thread(target=self._threaded_camera_check, daemon=True)
        self.camera_check_thread.start()
        self.root.after(100, self._update_camera_status_from_thread)

    def _threaded_camera_check(self):
        if self.camera:
            self.camera_check_result = self.camera.get_camera_summary()
        else:
            self.camera_check_result = "Error: Camera controller not initialized."

    def _update_camera_status_from_thread(self):
        if self.camera_check_thread and not self.camera_check_thread.is_alive():
            result_text = self.camera_check_result
            self.camera_status_var.set(result_text)
            if "Connected" in result_text: self.camera_status_label.config(foreground='green')
            elif "No Camera Found" in result_text or "Error" in result_text: self.camera_status_label.config(foreground='red')
            if self.camera_trigger_mode_var.get() == "USB Control":
                 self.check_camera_button.config(state=tkinter.NORMAL)
        else:
            self.root.after(100, self._update_camera_status_from_thread)

    def on_camera_mode_change(self, event=None):
        if self.camera:
            self.camera.set_trigger_mode(self.camera_trigger_mode_var.get())
        is_usb = self.camera_trigger_mode_var.get() == "USB Control"
        if self.check_camera_button: self.check_camera_button.config(state=tkinter.NORMAL if is_usb else tkinter.DISABLED)
        if self.take_preview_button: self.take_preview_button.config(state=tkinter.NORMAL if is_usb else tkinter.DISABLED)
        if is_usb:
            self.camera_status_var.set("Status: Unknown")
            self.camera_status_label.config(foreground='black')
        else:
            self.camera_status_var.set("Status: S2 Cable Mode")
            self.camera_status_label.config(foreground='blue')

    def toggle_holygrail_mode(self):
        is_hg_enabled = self.holygrail_enabled_var.get()
        state = tkinter.DISABLED if is_hg_enabled else tkinter.NORMAL
        self.interval_entry.config(state=state)
        self.exposure_pause_entry.config(state=state)
        self.timed_radio.config(state=state)
        self.manual_radio.config(state=state)
        if is_hg_enabled:
            self.camera_trigger_mode_var.set("USB Control")
            self.on_camera_mode_change()
            self.camera_trigger_combo.config(state=tkinter.DISABLED)
            logging.info("Holy Grail mode enabled. Interval/Exposure Pause disabled. Trigger forced to USB.")
        else:
            self.camera_trigger_combo.config(state="readonly")
            logging.info("Holy Grail mode disabled.")
            
    def update_rotation_direction(self, event=None):
        self.rotation_direction_gpio = GPIO_HIGH if self.rotation_direction_var.get()=="CW" else GPIO_LOW
        logging.info(f"Rotation dir UI: {self.rotation_direction_var.get()}, GPIO: {'H' if self.rotation_direction_gpio==GPIO_HIGH else 'L'}")

    def update_mode(self):
        self.manual_mode = (self.mode_var.get() == "manual")
        if not self.holygrail_enabled_var.get():
            self.interval_entry.config(state=tkinter.DISABLED if self.manual_mode else tkinter.NORMAL)
        logging.info(f"Mode set: {'Manual' if self.manual_mode else 'Timed'}")

    def manual_trigger_action(self, event=None):
        if self.manual_mode and self.gantry_running and not self.motors_moving:
            if self.photos_taken < self.num_photos_var.get(): self.force_interval = True; logging.info("Manual trigger: forcing interval.")
            else: logging.info("Manual trigger: max photos reached.")
        elif not self.manual_mode: logging.warning("Manual trigger: Not in manual mode.")
        elif not self.gantry_running: logging.warning("Manual trigger: Gantry not running.")
        elif self.motors_moving: logging.warning("Manual trigger: Motors moving.")

    def calculate_distribution_arrays(self):
        try:
            length_mm = int(self.length_var.get() or "0")
            min_speed_mm = self.minimum_slider_speed_var.get()
            num_photos = self.num_photos_var.get()
            if num_photos <= 0: raise ValueError("Number of photos must be > 0.")
        except (ValueError, tkinter.TclError) as e:
            messagebox.showerror("Error", f"Invalid numeric input: {e}")
            logging.error(f"Calculation input error: {e}")
            return False

        slider_micro = self.hardware.slider_microstepping if self.hardware else 4
        dist_pulse = (BELT_PITCH * PULLEY_TEETH) / (STEPS_PER_REVOLUTION_SLIDER * slider_micro) if slider_micro > 0 else 0
        self.total_steps = int(length_mm / dist_pulse) if dist_pulse > 0 else 0

        min_speed_pulses = int(min_speed_mm / dist_pulse) if dist_pulse > 0 else 0
        total_min_speed_pulses = min_speed_pulses * num_photos

        slider_func_name = self.distribution_var.get()
        slider_func = self.distribution_functions.get(slider_func_name, self.even_distribution)
        if slider_func_name == "object_tracking": slider_func = self.even_distribution

        if total_min_speed_pulses >= self.total_steps and self.total_steps > 0:
            messagebox.showinfo("Input Info", "The 'Minimum Speed' requires more travel than the total 'Length'.\n\nTotal length will be prioritized.")
            logging.warning(f"Min speed req ({total_min_speed_pulses} steps) exceeds total length ({self.total_steps} steps). Prioritizing length.")
            slider_raw = slider_func(num_photos, self.total_steps, 0)
            self.steps_distribution = self.scale_array_to_sum(slider_raw, self.total_steps).astype(int)
        else:
            curve_slider_pulses = self.total_steps - total_min_speed_pulses
            slider_raw = slider_func(num_photos, curve_slider_pulses, 0)
            base_distribution = self.scale_array_to_sum(slider_raw, curve_slider_pulses).astype(int)
            self.steps_distribution = base_distribution + min_speed_pulses

        if self.total_steps > 0 and np.sum(self.steps_distribution) != self.total_steps and num_photos > 0 and self.steps_distribution.size > 0:
            error = self.total_steps - np.sum(self.steps_distribution)
            self.steps_distribution[-1] += error
            logging.debug(f"Applied rounding correction of {error} to last slider step.")

        rot_angle = self.rotation_angle_var.get()
        total_rot_pulses = int((rot_angle / 360.0) * STEPS_PER_REVOLUTION_ROTATION)
        rot_func_name = self.rotation_distribution_var.get()
        rot_func = self.distribution_functions.get(rot_func_name)
        if not rot_func: return False

        try:
            value_for_rot_func = rot_angle if rot_func_name == "object_tracking" else total_rot_pulses
            rot_raw = rot_func(num_photos, value_for_rot_func, 0)
        except Exception as e:
            logging.error(f"Rotation distribution function error '{rot_func_name}': {e}", exc_info=True)
            return False

        rot_raw = np.zeros(num_photos) if not isinstance(rot_raw, np.ndarray) or (rot_raw.size == 0 and num_photos > 0) else rot_raw
        if rot_func_name == "object_tracking": rot_final = rot_raw.astype(int)
        else: rot_final = self.scale_array_to_sum(rot_raw, total_rot_pulses).astype(int)
        if total_rot_pulses > 0 and np.sum(rot_final) != total_rot_pulses and num_photos > 0 and rot_final.size > 0:
            error = total_rot_pulses - np.sum(rot_final)
            rot_final[-1] += error
            logging.debug(f"Applied rounding correction of {error} to last rotation step.")

        logging.info(f"Slider Steps (Total:{self.total_steps}): Sum={np.sum(self.steps_distribution)}, Arr={self.steps_distribution.tolist()}")
        logging.info(f"Rot Steps (UI Pulses:{total_rot_pulses}): Sum={np.sum(rot_final)}, Arr={rot_final.tolist()}")
        self.combined_intervals = [{'slider_steps': int(s), 'rotation_steps': int(r)} for s, r in zip(self.steps_distribution, rot_final)]

        return True

    def start_timelapse(self):
        logging.info("Start Timelapse pressed.")
        if self.gantry_running: logging.warning("Already running."); return
        if not self.hardware: messagebox.showerror("Error", "Hardware N/A."); return
        if not self.validate_inputs() or not self.calculate_distribution_arrays() or not self.combined_intervals: return
        self.update_ui_for_run_state(True)
        self.gantry_running = True; self.stop_flag.clear(); self.photos_taken = 0
        self.force_interval = False; self.aux_trigger_active = False

        self.last_aux_pin_state = -1
        if self.hardware:
            try:
                self.last_aux_pin_state = GPIO.input(AUX_TRIGGER_PIN)
            except Exception as e:
                logging.error(f"AUX pin read error on start: {e}")

        self.skip_next_scheduled_shot_due_to_aux = False
        self.photo_count_label.config(text=f"Photos: 0 / {self.num_photos_var.get()}")
        self.time_remaining_label.config(text="Time Rem: Estimating...")
        length_mm = int(self.length_var.get() or "0")

        def timelapse_init_and_run():
            thread_name = threading.current_thread().name
            logging.info(f"Timelapse Thread ({thread_name}) started.")
            init_ok = True
            try:
                if self.holygrail_enabled_var.get():
                    self.holygrail_controller = HolyGrailController(
                        lat=float(self.holygrail_latitude_var.get()),
                        lon=float(self.holygrail_longitude_var.get()),
                        settings_table_str=self.holygrail_settings_text.get("1.0", tkinter.END)
                    )

                if length_mm > 0:
                    homing_dir_high = (self.user_direction_ui.get() == "left")
                    logging.info(f"Init for timelapse '{self.user_direction_ui.get()}'. Homing DIR HIGH: {homing_dir_high}.")
                    self.hardware.move_to_end(homing_dir_high)
                    time.sleep(MOVEMENT_BUFFER)
                    if self.stop_flag.is_set(): logging.warning("Init stopped."); init_ok = False

                if self.stop_flag.is_set(): init_ok = False

                if init_ok:
                    is_low_power = self.low_power_mode_var.get()
                    logging.info(f"Timelapse run: Applying power mode (low power: {is_low_power}).")
                    self.hardware.set_power_mode(low_power_active=is_low_power)
                    self.run_timelapse_logic()

            except Exception as e_thread:
                 logging.error(f"Unhandled error in {thread_name} thread: {e_thread}", exc_info=True)
                 messagebox.showerror("Timelapse Error", f"An error occurred: {e_thread}")
                 self.stop_flag.set()
            finally:
                if self.hardware and self.hardware.current_power_mode_is_low:
                    logging.info(f"{thread_name} end: Was in low power, reverting to full power.")
                    self.hardware.set_power_mode(low_power_active=False)
                self.holygrail_controller = None
                logging.info(f"Timelapse Thread ({thread_name}) finished execution.")

        self.timelapse_thread = threading.Thread(target=timelapse_init_and_run, daemon=True, name="TimelapseThread")
        self.timelapse_thread.start()
        self.monitor_operation_thread(self.timelapse_thread)

    def start_preview(self):
        logging.info("Run Preview pressed.")
        if self.gantry_running: logging.warning("Already running."); return
        if not self.hardware: messagebox.showerror("Error", "Hardware N/A."); return
        if not self.validate_inputs() or not self.calculate_distribution_arrays() or not self.combined_intervals: return
        self.update_ui_for_run_state(True)
        self.gantry_running = True; self.stop_flag.clear()
        length_mm = int(self.length_var.get() or "0")

        def preview_init_and_run():
            thread_name = threading.current_thread().name
            logging.info(f"Preview Thread ({thread_name}) started.")
            init_ok = True
            try:
                if length_mm > 0:
                    homing_dir_high = (self.user_direction_ui.get() == "left")
                    logging.info(f"Init for preview '{self.user_direction_ui.get()}'. Homing DIR HIGH: {homing_dir_high}.")
                    self.hardware.move_to_end(homing_dir_high); time.sleep(MOVEMENT_BUFFER)
                    if self.stop_flag.is_set(): logging.warning("Init stopped."); init_ok = False

                if self.stop_flag.is_set(): init_ok = False

                if init_ok:
                    is_low_power = self.low_power_mode_var.get()
                    logging.info(f"Preview run: Applying power mode (low power: {is_low_power}).")
                    self.hardware.set_power_mode(low_power_active=is_low_power)
                    self.run_preview_logic()
            except Exception as e_thread:
                logging.error(f"Unhandled error in {thread_name} thread: {e_thread}", exc_info=True)
                self.stop_flag.set()
            finally:
                if self.hardware and self.hardware.current_power_mode_is_low:
                     logging.info(f"{thread_name} end: Was in low power, reverting to full power.")
                     self.hardware.set_power_mode(low_power_active=False)
                logging.info(f"Preview Thread ({thread_name}) finished execution.")

        self.timelapse_thread = threading.Thread(target=preview_init_and_run, daemon=True, name="PreviewThread")
        self.timelapse_thread.start()
        self.monitor_operation_thread(self.timelapse_thread)
        
    def monitor_operation_thread(self, thread_to_monitor):
        if thread_to_monitor.is_alive():
            self.root.after(100, lambda: self.monitor_operation_thread(thread_to_monitor))
        else:
            logging.info(f"Operation thread {thread_to_monitor.name} has finished.")
            self.gantry_running = False; self.motors_moving = False
            if not self.root.winfo_exists(): return
            self.update_ui_for_run_state(False)

            current_label_text = ""
            try:
                if self.time_remaining_label and self.time_remaining_label.winfo_exists():
                    current_label_text = self.time_remaining_label.cget("text")
            except tkinter.TclError: return

            if not ("Post-Action" in current_label_text or "Done" in current_label_text or "Stopped" in current_label_text):
                final_msg = "Time Rem: Finished"
                if thread_to_monitor.name == "TimelapseThread":
                    max_photos = self.num_photos_var.get() if self.num_photos_var else -1
                    if self.stop_flag.is_set(): final_msg = "Time Rem: Stopped"
                    elif max_photos > 0 and self.photos_taken >= max_photos: final_msg = "Time Rem: Done"
                if self.time_remaining_label and self.time_remaining_label.winfo_exists():
                    self.time_remaining_label.config(text=final_msg)

    def run_timelapse_logic(self):
        try:
            total_photos_to_take = self.num_photos_var.get()
            interval_s = self.interval_var.get()
            exposure_pause_s = self.exposure_pause_var.get()
            slider_timelapse_dir_pin_is_high = (self.user_direction_ui.get() == "right")
            length_mm = int(self.length_var.get() or "0")
            post_action_choice = self.post_timelapse_position_var.get()
            homing_dir_for_timelapse_start_is_high = (self.user_direction_ui.get() == "left")
            is_hg_mode = self.holygrail_enabled_var.get()
            
            min_aperture = self.holygrail_min_aperture_var.get() if is_hg_mode else 0
            max_aperture = self.holygrail_max_aperture_var.get() if is_hg_mode else 0
            base_iso = self.holygrail_base_iso_var.get() if is_hg_mode else 0
            max_iso = self.holygrail_max_iso_var.get() if is_hg_mode else 0

        except (tkinter.TclError, ValueError) as e:
            logging.error(f"Could not read UI values at start of timelapse thread: {e}", exc_info=True)
            self.stop_flag.set(); return

        try:
            last_exposure_duration = 0
            
            for i in range(total_photos_to_take):
                if self.stop_flag.is_set(): break
                start_time = time.time()
                
                if is_hg_mode and self.holygrail_controller:
                    settings = self.holygrail_controller.get_current_settings(
                        min_aperture, max_aperture, base_iso, max_iso
                    )
                    self.camera.set_aperture(settings['aperture'])
                    self.camera.set_iso(settings['iso'])
                    self.camera.set_white_balance_kelvin(settings['kelvin'])
                    
                    if settings['mode'] == 'bulb':
                        self.camera.set_shutter_speed('bulb')
                        self.camera.open_shutter()
                        time.sleep(settings['bulb_duration'])
                        self.camera.close_shutter()
                        self.photos_taken += 1
                        last_exposure_duration = settings['bulb_duration']
                    else:
                        self.camera.set_shutter_speed(settings['shutter'])
                        if self.camera.trigger_photo():
                            self.photos_taken += 1
                        else:
                            logging.error("HG Mode: Camera trigger failed! Stopping."); self.stop_flag.set(); break
                        try: last_exposure_duration = float(eval(str(settings['shutter'])))
                        except: last_exposure_duration = 1.0
                else:
                    if self.camera.trigger_photo(): self.photos_taken += 1
                    else: logging.error("Standard Mode: Camera trigger failed! Stopping."); self.stop_flag.set(); break
                    time.sleep(exposure_pause_s)

                if self.stop_flag.is_set(): break
                if self.root.winfo_exists(): self.photo_count_label.config(text=f"Photos: {self.photos_taken}/{total_photos_to_take}")

                self.motors_moving = True
                interval_data = self.combined_intervals[i] if i < len(self.combined_intervals) else {'slider_steps':0, 'rotation_steps':0}
                slider_steps = interval_data.get('slider_steps', 0)
                rotation_steps = interval_data.get('rotation_steps', 0)

                if length_mm > 0 and slider_steps > 0:
                    if self.hardware.end_switch_triggered_by_hw(): self.stop_flag.set(); break
                    self.hardware.set_slider_step(slider_steps, slider_timelapse_dir_pin_is_high)
                
                if self.stop_flag.is_set(): break
                
                if rotation_steps != 0:
                    actual_rot = abs(rotation_steps); eff_dir = (self.rotation_direction_gpio == GPIO.HIGH) != (rotation_steps < 0)
                    self.hardware.move_steps(ROTATION_EN_PIN,ROTATION_DIR_PIN,ROTATION_STEP_PIN, actual_rot, eff_dir)
                
                self.motors_moving = False
                if self.stop_flag.is_set(): break
                
                if self.manual_mode and not is_hg_mode:
                    self.force_interval = False
                    while not self.force_interval and not self.stop_flag.is_set(): time.sleep(0.1)
                    continue

                time_elapsed = time.time() - start_time
                if is_hg_mode:
                    dynamic_interval = last_exposure_duration + 2.0
                    wait_time = dynamic_interval - time_elapsed
                else:
                    wait_time = interval_s - time_elapsed
                
                if wait_time > 0: self.stop_flag.wait(wait_time)

                if self.root.winfo_exists():
                    rem_photos = total_photos_to_take - self.photos_taken
                    avg_interval = (interval_s if not is_hg_mode else (last_exposure_duration + 2.0))
                    rem_s = rem_photos * avg_interval
                    self.time_remaining_label.config(text=f"Time Rem: {self.format_time(rem_s)}")
            
            if not self.stop_flag.is_set() and post_action_choice != "Do Nothing" and length_mm > 0:
                 if self.root.winfo_exists(): self.time_remaining_label.config(text=f"Post-Action...")
                 if post_action_choice == "Return to Start": self.hardware.move_to_end(homing_dir_for_timelapse_start_is_high)
                 elif post_action_choice == "Move to End": self.hardware.move_to_end(not homing_dir_for_timelapse_start_is_high)

        except Exception as e:
            logging.error(f"Error in timelapse logic: {e}", exc_info=True)
            if self.root.winfo_exists(): messagebox.showerror("Timelapse Error", f"An error occurred: {e}")
            self.stop_flag.set()

    def run_preview_logic(self):
        try:
            total_intervals = len(self.combined_intervals)
            slider_preview_dir_pin_is_high = (self.user_direction_ui.get() == "right")
            preview_delay_s = 0.05
            length_mm = int(self.length_var.get() or "0")
            for i, interval_data_from_list in enumerate(self.combined_intervals):
                if self.stop_flag.is_set(): logging.info("Preview stopped."); break
                if self.root.winfo_exists(): self.photo_count_label.config(text=f"Preview Step: {i+1} / {total_intervals}")
                self.motors_moving = True
                
                slider_steps_for_interval = interval_data_from_list.get('slider_steps', 0)
                rotation_steps_for_interval = interval_data_from_list.get('rotation_steps', 0)

                if length_mm > 0 and slider_steps_for_interval > 0:
                    if self.hardware.end_switch_triggered_by_hw(): self.stop_flag.set(); break
                    self.hardware.set_slider_step(slider_steps_for_interval, slider_preview_dir_pin_is_high)
                    if self.hardware.end_switch_triggered_by_hw(): self.stop_flag.set(); break
                if self.stop_flag.is_set(): break
                
                if rotation_steps_for_interval != 0:
                    actual_rot_steps = abs(rotation_steps_for_interval)
                    eff_rot_dir_high = (self.rotation_direction_gpio == GPIO.HIGH) != (rotation_steps_for_interval < 0)
                    self.hardware.move_steps(ROTATION_EN_PIN, ROTATION_DIR_PIN, ROTATION_STEP_PIN, actual_rot_steps, eff_rot_dir_high)
                if self.stop_flag.is_set(): break
                
                self.motors_moving = False
                self.stop_flag.wait(preview_delay_s)
                if self.stop_flag.is_set(): break
            logging.info("Preview logic execution finished.")
        except Exception as e: logging.error(f"Error in preview logic: {e}", exc_info=True)

    def request_stop_operations(self):
        logging.info("Stop requested by user via button.")
        self.stop_flag.set()
        if self.stop_button and self.stop_button.winfo_exists(): self.stop_button.config(state=tkinter.DISABLED)

    def on_closing(self):
        logging.info("Window close requested.")
        if self.gantry_running:
            if messagebox.askyesno("Confirm Exit", "Operation in progress. Stop and exit?"):
                self.stop_flag.set()
                if self.timelapse_thread and self.timelapse_thread.is_alive():
                    self.timelapse_thread.join(timeout=15.0)
                self.cleanup_and_exit_app()
        else:
            self.cleanup_and_exit_app()

    def cleanup_and_exit_app(self):
        logging.info("Initiating application cleanup and exit.")
        self.stop_flag.set()
        if self.hardware:
            self.hardware.cleanup()
            self.hardware = None
        if self.root and self.root.winfo_exists(): self.root.destroy()
        logging.info("Application exit complete.")

    def validate_inputs(self):
        try:
            int(self.length_var.get() or "0"); float(self.rotation_angle_var.get() or "0.0")
            num_p = self.num_photos_var.get(); ival = self.interval_var.get(); exp_p = self.exposure_pause_var.get(); min_s = self.minimum_slider_speed_var.get()
            if num_p <= 0: raise ValueError("Num photos must be > 0.")
            if not self.manual_mode and not self.holygrail_enabled_var.get() and ival <= 0: raise ValueError("Interval must be > 0.")
            if exp_p < 0: raise ValueError("Exposure pause cannot be negative.")
            if min_s < 0: raise ValueError("Min slider speed cannot be negative.")
            
            if self.holygrail_enabled_var.get():
                float(self.holygrail_latitude_var.get())
                float(self.holygrail_longitude_var.get())
                min_ap = self.holygrail_min_aperture_var.get()
                max_ap = self.holygrail_max_aperture_var.get()
                if min_ap <= 0 or max_ap <= 0: raise ValueError("Aperture must be > 0.")
                int(self.holygrail_base_iso_var.get())
                int(self.holygrail_max_iso_var.get())
                json.loads(self.holygrail_settings_text.get("1.0", tkinter.END))
            return True
        except (ValueError, tkinter.TclError, json.JSONDecodeError) as e:
            messagebox.showerror("Input Error", f"Invalid input value: {e}")
            logging.error(f"Validation failed: {e}")
            return False

    def update_distribution_image(self, event=None):
        key = self.distribution_var.get()
        image_name = self.distribution_images.get(key)
        
        if image_name:
            try:
                image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), image_name)
                
                img = Image.open(image_path)
                img.thumbnail((250, 100), Image.Resampling.LANCZOS)
                
                self.distribution_photo_image = ImageTk.PhotoImage(img)
                self.distribution_image_label.config(image=self.distribution_photo_image)
            except FileNotFoundError:
                self.distribution_image_label.config(image='', text=f"Image not found:\n{image_name}")
            except Exception as e:
                logging.error(f"Error loading distribution image {image_name}: {e}")
                self.distribution_image_label.config(image='', text="Error loading image")
        else:
            self.distribution_image_label.config(image='', text="")

    def bind_keys(self):
        self.root.bind("<Key>", self.handle_key_event)

    def on_entry_focus(self, event):
        self.entry_has_focus = True
    def on_entry_defocus(self, event):
        self.entry_has_focus = False

    def handle_key_event(self, event):
        if self.entry_has_focus: return

        key_map = {
            "KP_Multiply": self.rotate_ccw_manual, "asterisk": self.rotate_ccw_manual,
            "KP_Divide": self.rotate_cw_manual, "slash": self.rotate_cw_manual,
            "KP_Add": self.gantry_right_manual, "plus": self.gantry_right_manual, "equal": self.gantry_right_manual,
            "KP_Subtract": self.gantry_left_manual, "minus": self.gantry_left_manual,
            "KP_Enter": self.manual_trigger_action, "Return": self.manual_trigger_action
        }
        action = key_map.get(event.keysym)
        if action:
            if action == self.manual_trigger_action:
                action()
            elif not self.gantry_running:
                action()
            else:
                logging.warning(f"Key action ignored, gantry running.")

    def format_time(self, seconds):
        if seconds < 0: seconds = 0
        m, s = divmod(seconds, 60)
        h, m = divmod(m, 60)
        if h > 0: return f"{int(h)}h {int(m):02d}m {int(s):02d}s"
        elif m > 0: return f"{int(m):02d}m {int(s):02d}s"
        else: return f"{int(s):02d}s"

    def _manual_move_wrapper(self, move_func_name, *args):
        if self.gantry_running or not self.hardware or self.motors_moving: logging.warning(f"Manual move {move_func_name} ignored."); return
        self.motors_moving = True
        try: getattr(self.hardware, move_func_name)(*args)
        except Exception as e: logging.error(f"Manual move {move_func_name} error: {e}", exc_info=True)
        finally: self.motors_moving = False

    def rotate_ccw_manual(self): self._manual_move_wrapper("rotate_motor", 1, (self.rotation_direction_var.get() == "CCW"))
    def rotate_cw_manual(self): self._manual_move_wrapper("rotate_motor", 1, (self.rotation_direction_var.get() == "CW"))
    def gantry_right_manual(self):
        dist_per_pulse = (BELT_PITCH * PULLEY_TEETH) / (STEPS_PER_REVOLUTION_SLIDER * (self.hardware.slider_microstepping if self.hardware else 4))
        steps = int(1 / dist_per_pulse) if dist_per_pulse > 0 else 50
        self._manual_move_wrapper("set_slider_step", steps, True)
    def gantry_left_manual(self):
        dist_per_pulse = (BELT_PITCH * PULLEY_TEETH) / (STEPS_PER_REVOLUTION_SLIDER * (self.hardware.slider_microstepping if self.hardware else 4))
        steps = int(1 / dist_per_pulse) if dist_per_pulse > 0 else 50
        self._manual_move_wrapper("set_slider_step", steps, False)

    def catenary_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-1.5, 1.5, total_intervals); curve = np.cosh(x)
        return curve - np.min(curve) if curve.size > 0 else curve
    def inverted_catenary_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        s = self.catenary_distribution(total_intervals,0,0)
        return (np.max(s) - s) if s.size > 0 and np.max(s) > 0 else np.ones(total_intervals)
    def gaussian_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-2.5, 2.5, total_intervals); curve = np.exp(-x**2 / 2)
        return curve - np.min(curve) if curve.size > 0 else curve
    def inverted_gaussian_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        s = self.gaussian_distribution(total_intervals,0,0)
        return (np.max(s) - s) if s.size > 0 and np.max(s) > 0 else np.ones(total_intervals)
    def ellipsoidal_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-1, 1, total_intervals); curve = (1 - x**2)**0.5; curve[np.isnan(curve)] = 0
        return curve - np.min(curve) if curve.size > 0 else curve
    def inverted_ellipsoidal_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        s = self.ellipsoidal_distribution(total_intervals,0,0)
        return (np.max(s) - s) if s.size > 0 and np.max(s) > 0 else np.ones(total_intervals)
    def parabolic_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-1, 1, total_intervals); curve = 1 - x**2
        return curve - np.min(curve) if curve.size > 0 else curve
    def inverted_parabolic_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        s = self.parabolic_distribution(total_intervals,0,0)
        return (np.max(s) - s) if s.size > 0 and np.max(s) > 0 else np.ones(total_intervals)
    def cycloid_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(0, 2 * np.pi, total_intervals); curve = (1 - np.cos(x)) / 2
        return curve - np.min(curve) if curve.size > 0 else curve
    def inverted_cycloid_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        s = self.cycloid_distribution(total_intervals,0,0)
        return (np.max(s) - s) if s.size > 0 and np.max(s) > 0 else np.ones(total_intervals)
    def lame_curve_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg, n_param=4):
        if total_intervals <= 0: return np.array([])
        x = np.linspace(-1, 1, total_intervals); curve = (1 - np.abs(x)**n_param)**(1/n_param); curve[np.isnan(curve)] = 0
        return curve - np.min(curve) if curve.size > 0 else curve
    def inverted_lame_curve_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        s = self.lame_curve_distribution(total_intervals,0,0)
        return (np.max(s) - s) if s.size > 0 and np.max(s) > 0 else np.ones(total_intervals)
    def linear_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        return np.linspace(0.01, 1, total_intervals)
    def inverted_linear_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        return np.linspace(1, 0.01, total_intervals)
    def even_distribution(self, total_intervals, total_value_to_distribute, total_rotation_angle_deg):
        if total_intervals <= 0: return np.array([])
        return np.ones(total_intervals)
    def object_tracking_rotation(self, total_intervals, total_rotation_angle_deg_param, total_value_to_distribute_ignored):
        if total_intervals <= 0 or not hasattr(self, 'steps_distribution') or \
           self.steps_distribution.size != total_intervals or not self.hardware:
            logging.warning("Object tracking: Invalid inputs or slider steps/hardware not ready.")
            return np.zeros(total_intervals)

        if self.total_steps == 0 :
            logging.info("Object tracking: No slider movement. Treating as 'even' panoramic rotation if angle set.")
            angle_pulses = int((total_rotation_angle_deg_param / 360.0) * STEPS_PER_REVOLUTION_ROTATION)
            return self.scale_array_to_sum(np.ones(total_intervals), angle_pulses) if total_intervals > 0 and angle_pulses != 0 else np.zeros(total_intervals)

        slider_microsteps = self.hardware.slider_microstepping
        dist_per_pulse = (BELT_PITCH * PULLEY_TEETH) / (STEPS_PER_REVOLUTION_SLIDER * slider_microsteps) if slider_microsteps > 0 else 0
        if dist_per_pulse == 0: logging.warning("Object tracking: dist_per_pulse is zero."); return np.zeros(total_intervals)

        slider_move_mm_interval = self.steps_distribution * dist_per_pulse
        total_slider_travel_mm = self.total_steps * dist_per_pulse
        
        dist_to_obj_mm = total_slider_travel_mm
        if abs(total_rotation_angle_deg_param) > 1e-3 :
            tan_half_angle = math.tan(math.radians(abs(total_rotation_angle_deg_param)) / 2.0)
            if abs(tan_half_angle) > 1e-9:
                 dist_to_obj_mm = (total_slider_travel_mm / 2.0) / tan_half_angle
            else: dist_to_obj_mm = total_slider_travel_mm * 1000
        else: dist_to_obj_mm = total_slider_travel_mm * 1000
        if dist_to_obj_mm <= 1e-3:
            logging.warning(f"Object tracking: Dist to object very small ({dist_to_obj_mm:.2f}mm). Clamping.")
            dist_to_obj_mm = max(total_slider_travel_mm * 1000, 1000.0)

        cumulative_slider_pos_mm = 0.0
        initial_x_offset = -total_slider_travel_mm / 2.0
        previous_angle_rad = math.atan2(initial_x_offset, dist_to_obj_mm)
        rotation_angles_rad_interval = np.zeros(total_intervals)

        for i in range(total_intervals):
            cumulative_slider_pos_mm += slider_move_mm_interval[i]
            current_x_offset = cumulative_slider_pos_mm - (total_slider_travel_mm / 2.0)
            target_angle_rad = math.atan2(current_x_offset, dist_to_obj_mm)
            angle_change_rad = target_angle_rad - previous_angle_rad
            rotation_angles_rad_interval[i] = angle_change_rad
            previous_angle_rad = target_angle_rad

        pulses_per_radian = STEPS_PER_REVOLUTION_ROTATION / (2 * math.pi)
        rotation_pulses_interval_float = rotation_angles_rad_interval * pulses_per_radian
        
        geom_total_angle_deg = math.degrees(np.sum(rotation_angles_rad_interval))
        if np.sign(geom_total_angle_deg) != np.sign(total_rotation_angle_deg_param) and \
           abs(geom_total_angle_deg) > 1e-2 and abs(total_rotation_angle_deg_param) > 1e-2:
            logging.warning(f"Object Tracking: Geom rot dir ({geom_total_angle_deg:.1f}°) differs from UI target dir ({total_rotation_angle_deg_param:.1f}°). Using geometric profile.")
        
        if abs(total_rotation_angle_deg_param) < 1e-3 and abs(geom_total_angle_deg) > 1e-2:
            logging.info("Object tracking: UI angle zero, but geom calculated rotation. Setting rotation to zero.")
            return np.zeros(total_intervals)

        logging.debug(f"Object tracking raw pulses (float): {rotation_pulses_interval_float.tolist()}")
        return rotation_pulses_interval_float

    def scale_array_to_sum(self, arr_in, target_sum_val):
        arr = np.nan_to_num(np.array(arr_in, dtype=float)); target_sum = float(target_sum_val)
        if arr.size == 0: return np.array([], dtype=int)
        current_sum = np.sum(arr)
        if abs(current_sum) < 1e-9:
            if arr.size > 0:
                base, rem = divmod(int(round(target_sum)), arr.size)
                scaled = np.full(arr.size, base, dtype=int)
                for i in range(int(rem)): scaled[i % arr.size] += 1
                return scaled
            return np.array([], dtype=int)
        scaled_float = arr * (target_sum / current_sum)
        int_arr = np.floor(scaled_float).astype(int)
        remainders = scaled_float - int_arr
        error_to_dist = int(round(target_sum)) - np.sum(int_arr)
        if error_to_dist != 0 and arr.size > 0:
            if error_to_dist > 0:
                indices = np.argsort(remainders)[-error_to_dist:]
                for idx in indices: int_arr[idx] += 1
            elif error_to_dist < 0:
                indices = np.argsort(remainders)[:abs(error_to_dist)]
                for idx in indices: int_arr[idx] -= 1
        final_adj = int(round(target_sum)) - np.sum(int_arr)
        if final_adj!=0 and int_arr.size>0: int_arr[-1]+=final_adj
        return int_arr

def main():
    root = None; app = None
    try:
        root = tkinter.Tk()
        app = PiSliderApp(root)
        if app.hardware: root.mainloop(); logging.info("Tkinter mainloop finished.")
        else: logging.error("App hardware None, root likely destroyed or error shown.")
    except Exception as e:
        logging.critical(f"Main startup error: {e}", exc_info=True)
        if root and root.winfo_exists(): messagebox.showerror("Fatal Startup Error", f"Critical error: {e}\nExit.")
    finally:
        logging.info("--- Main: App exiting ---")
        if 'GPIO' in globals() and hasattr(GPIO, 'getmode') and GPIO.getmode() is not None:
            logging.warning("Fallback GPIO cleanup in main finally.")
            try: GPIO.cleanup(); logging.info("Fallback GPIO cleanup ok.")
            except Exception as e: logging.error(f"Fallback GPIO cleanup error: {e}")

if __name__ == "__main__":
    main()
