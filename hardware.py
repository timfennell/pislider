import RPi.GPIO as GPIO
import time
import serial
import logging
import subprocess
import os
from enum import Enum
import threading

# --- Configuration Constants ---
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
UART_PORT = '/dev/serial0'
UART_BAUDRATE = 115200
GPIO_HIGH = GPIO.HIGH
GPIO_LOW = GPIO.LOW

class MotorState(Enum):
    DISABLED = 0
    SLIDER_ENABLED = 1
    ROTATION_ENABLED = 2
    BOTH_ENABLED = 3

class CameraController:
    """Handles all camera interaction, with robust retry and cooldown mechanisms."""
    def __init__(self, hardware_controller, lock):
        self.hardware = hardware_controller
        self.lock = lock
        self.trigger_mode = 'S2 Cable'
        
        try:
            subprocess.run(['killall', 'gphoto2'], capture_output=True, text=True)
            logging.info("Ran pre-emptive 'killall gphoto2' to ensure clean state.")
        except FileNotFoundError:
            logging.warning("'killall' command not found. Cannot pre-emptively kill processes.")
        
        try:
            self.gphoto2_available = self.check_gphoto2()
        except Exception as e:
            logging.error(f"Initial gphoto2 check failed after multiple attempts. USB control will be disabled. Error: {e}")
            self.gphoto2_available = False

        logging.info(f"gphoto2 availability: {self.gphoto2_available}")
        if self.gphoto2_available:
            logging.info("USB control is available. Ensure your camera is set to 'PC Remote'.")

    def _run_gphoto2_command(self, command, timeout=15, retries=2, retry_delay=2):
        with self.lock:
            process = None
            for attempt in range(retries + 1):
                try:
                    subprocess.run(
                        ['gphoto2', '--auto-detect'],
                        capture_output=True, text=True, timeout=10, check=False
                    )
                    time.sleep(0.1)
                    
                    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid)
                    stdout, stderr = process.communicate(timeout=timeout)
                    
                    if process.returncode == 0:
                        return stdout

                    logging.warning(f"gphoto2 command failed (attempt {attempt + 1}/{retries + 1}): {' '.join(command)}")
                    if stderr: logging.warning(f"gphoto2 stderr: {stderr.strip()}")
                    
                    if attempt < retries:
                        time.sleep(retry_delay)
                    else:
                        logging.error(f"gphoto2 command failed after {retries + 1} attempts.")
                        return None
                        
                except subprocess.TimeoutExpired:
                    logging.warning(f"gphoto2 command timed out: {' '.join(command)}")
                    if process:
                        try: os.killpg(os.getpgid(process.pid), 9)
                        except ProcessLookupError: pass
                    if attempt == retries:
                         logging.error("gphoto2 command failed due to timeout after all retries.")
                         return None
                    time.sleep(retry_delay)

                except FileNotFoundError:
                    logging.error("gphoto2 command not found. Please ensure it is installed and in your PATH.")
                    self.gphoto2_available = False
                    return None
                except Exception as e:
                    logging.error(f"An unexpected error occurred running gphoto2: {e}", exc_info=True)
                    return None
        return None

    def get_config_value(self, config_name):
        if not self.gphoto2_available: raise RuntimeError("gphoto2 not available")
        command = ['gphoto2', '--get-config', f"/{config_name}"]
        stdout = self._run_gphoto2_command(command, timeout=5)
        if stdout:
            for line in stdout.split('\n'):
                if line.startswith('Current:'): return line.split('Current:')[1].strip()
        raise RuntimeError(f"Could not get config value for '{config_name}'")

    def check_gphoto2(self):
        max_attempts = 5
        delay_between_attempts = 3
        for attempt in range(1, max_attempts + 1):
            logging.info(f"gphoto2 check, attempt {attempt}/{max_attempts}...")
            try:
                result = subprocess.run(['gphoto2', '--auto-detect'], capture_output=True, text=True, timeout=4, check=True)
                if "Model" in result.stdout and "Port" in result.stdout:
                    logging.info(f"gphoto2 check successful on attempt {attempt}.")
                    return True
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
                logging.warning(f"Attempt {attempt} failed: {e}. Retrying in {delay_between_attempts}s...")
                if attempt < max_attempts:
                    time.sleep(delay_between_attempts)
            except FileNotFoundError:
                logging.error("gphoto2 command not found. Please ensure it is installed.")
                raise
        
        raise RuntimeError("gphoto2 check failed after all attempts. Camera not detected or ready.")

    def set_trigger_mode(self, mode):
        self.trigger_mode = 'S2 Cable' if mode == 'USB Control' and not self.gphoto2_available else mode
        logging.info(f"Camera trigger mode set to: {self.trigger_mode}")

    def trigger_photo(self):
        if self.trigger_mode == 'S2 Cable':
            return self.hardware.trigger_camera_s2()
        elif self.trigger_mode == 'USB Control' and self.gphoto2_available:
            return self._run_gphoto2_command(['gphoto2', '--capture-image'], timeout=15) is not None
        return False

    def get_camera_summary(self):
        if not self.gphoto2_available: return "gphoto2 not found."
        stdout = self._run_gphoto2_command(['gphoto2', '--summary'], timeout=10)
        if not stdout: return "No Camera Found"
        for line in stdout.split('\n'):
            if 'Model:' in line: return f"Connected: {line.split('Model:')[1].strip()}"
        return "Unknown camera detected."
    
    def apply_settings(self, settings):
        if self.trigger_mode != 'USB Control': return False
        success = self._set_config("main/imgsettings/iso", settings['iso'])
        success &= self._set_config("main/capturesettings/f-number", settings['aperture'])
        success &= self.set_white_balance_kelvin(settings['kelvin'])
        success &= self._set_config("main/capturesettings/shutterspeed", settings['shutter'])
        return success

    def _set_config(self, config_path, value):
        if not self.gphoto2_available: return False
        result = self._run_gphoto2_command(['gphoto2', '--set-config-value', f"{config_path}={value}"], timeout=7)
        if result is None:
            logging.warning(f"FAILED to set '{config_path}' to '{value}'.")
            return False
        return True

    def set_white_balance_kelvin(self, kelvin_value):
        if self._set_config("main/imgsettings/whitebalance", "Choose Color Temperature"):
            return self._set_config("main/imgsettings/colortemperature", kelvin_value)
        return False

    def get_shutter_s(self, shutter_str):
        try:
            if '/' in shutter_str: num, den = shutter_str.split('/'); return float(num) / float(den)
            return float(shutter_str)
        except (ValueError, TypeError): return 1.0

    def capture_and_download(self, destination_path, download_preview_path=None):
        if not self.gphoto2_available: return False
        
        command = ['gphoto2', '--capture-image-and-download', '--force-overwrite', f'--filename={destination_path}']
        
        if download_preview_path:
            hook_script_path = os.path.join(os.path.dirname(__file__), "hook-script.sh")
            if not os.path.exists(hook_script_path):
                logging.error(f"Hook script not found at {hook_script_path}!")
                return False
                
            os.environ['PREVIEW_PATH'] = download_preview_path
            command.insert(2, f'--hook-script={hook_script_path}')
        
        result = self._run_gphoto2_command(command, timeout=120)
        
        if 'PREVIEW_PATH' in os.environ:
            del os.environ['PREVIEW_PATH']

        return result is not None and os.path.exists(destination_path) and os.path.getsize(destination_path) > 0

    def cleanup(self):
        logging.info("Cleaning up HardwareController resources...")
        try:
            if GPIO.getmode() is not None:
                self.hardware.disable_motors()
                GPIO.cleanup()
                logging.info("GPIO cleanup successful.")
        except Exception as e:
            logging.error(f"Error during GPIO cleanup: {e}")

class HardwareController:
    """Handles all low-level hardware interaction: GPIO and Motors."""
    def __init__(self):
        self.SLIDER_EN_PIN = SLIDER_EN_PIN; self.ROTATION_EN_PIN = ROTATION_EN_PIN
        self.SLIDER_DIR_PIN = SLIDER_DIR_PIN; self.ROTATION_DIR_PIN = ROTATION_DIR_PIN
        self.SLIDER_STEP_PIN = SLIDER_STEP_PIN; self.ROTATION_STEP_PIN = ROTATION_STEP_PIN
        self.slider_microstepping = 4; self.rotation_direction_gpio = GPIO.HIGH
        self.setup_gpio(); self.stop_flag = threading.Event()

    def setup_gpio(self):
        try:
            GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
            GPIO.setup([END_SWITCH_1_PIN, END_SWITCH_2_PIN, AUX_TRIGGER_PIN], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup([CAMERA_TRIGGER_PIN, SLIDER_DIR_PIN, SLIDER_STEP_PIN, ROTATION_DIR_PIN, ROTATION_STEP_PIN, SLIDER_EN_PIN, ROTATION_EN_PIN], GPIO.OUT, initial=GPIO.HIGH)
        except Exception as e:
            logging.error(f"Error configuring GPIO: {e}", exc_info=True)
            raise RuntimeError("Failed to configure GPIO pins.")

    def set_stop_flag_reference(self, stop_flag_event): self.stop_flag = stop_flag_event
    def enable_motors(self): GPIO.output([SLIDER_EN_PIN, ROTATION_EN_PIN], GPIO.LOW); time.sleep(0.1)
    def disable_motors(self): GPIO.output([SLIDER_EN_PIN, ROTATION_EN_PIN], GPIO.HIGH)

    def move_steps(self, dir_pin, step_pin, num_steps, is_forward):
        GPIO.output(dir_pin, GPIO_HIGH if is_forward else GPIO.LOW)
        delay = 0.0005 / (self.slider_microstepping / 4)
        for _ in range(abs(num_steps)):
            if self.stop_flag.is_set(): break
            GPIO.output(step_pin, GPIO.HIGH); time.sleep(delay)
            GPIO.output(step_pin, GPIO.LOW); time.sleep(delay)

    def set_slider_step(self, num_steps, is_forward):
        if (is_forward and GPIO.input(END_SWITCH_2_PIN) == GPIO.LOW) or \
           (not is_forward and GPIO.input(END_SWITCH_1_PIN) == GPIO.LOW):
            return
        self.move_steps(SLIDER_DIR_PIN, SLIDER_STEP_PIN, num_steps, is_forward)

    def rotate_motor(self, angle_deg, is_cw):
        num_steps = int(angle_deg * (48000 / 360))
        self.enable_motors()
        self.move_steps(ROTATION_DIR_PIN, ROTATION_STEP_PIN, num_steps, is_cw)
        self.disable_motors()

    def move_to_end(self, go_left):
        logging.info(f"Homing... Moving {'Left' if go_left else 'Right'}")
        self.enable_motors()
        direction, end_switch = (GPIO.LOW, END_SWITCH_1_PIN) if go_left else (GPIO.HIGH, END_SWITCH_2_PIN)
        GPIO.output(SLIDER_DIR_PIN, direction)
        for _ in range(300000):
            if GPIO.input(end_switch) == GPIO.LOW or self.stop_flag.is_set(): break
            self.move_steps(SLIDER_DIR_PIN, SLIDER_STEP_PIN, 1, True)
        else: logging.error("Homing failed: max steps reached.")
        if not self.stop_flag.is_set():
            logging.info("End switch hit. Backing off.")
            GPIO.output(SLIDER_DIR_PIN, not direction)
            self.move_steps(SLIDER_DIR_PIN, SLIDER_STEP_PIN, 500, True)
        self.disable_motors(); logging.info("Homing finished.")

    def trigger_camera_s2(self):
        GPIO.output(CAMERA_TRIGGER_PIN, GPIO.HIGH); time.sleep(0.2)
        GPIO.output(CAMERA_TRIGGER_PIN, GPIO.LOW); return True
