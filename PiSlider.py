import threading
import time
import tkinter as tk
from tkinter import ttk
import numpy as np
import RPi.GPIO as GPIO
import adafruit_motorkit

class PiSliderApp:
    def __init__(self, root):
        self.root = root
        self.direction_var = tk.StringVar(value='right')
        self.setup_gpio()
        self.create_widgets()
        self.gantry_running = False
        self.stop_flag = False
        self.factor = 1.0
        self.photos_taken = 0
        self.MOVEMENT_BUFFER = 1
        self.distribution_functions = {
            "catenary": self.catenary_distribution,
            "inverted_catenary": self.inverted_catenary_distribution,
            "gaussian": self.gaussian_distribution,
            "inverted_gaussian": self.inverted_gaussian_distribution,
            "ellipsoidal": self.ellipsoidal_distribution,
            "inverted_ellipsoidal": self.inverted_ellipsoidal_distribution,
            "parabolic": self.parabolic_distribution,
            "inverted_parabolic": self.inverted_parabolic_distribution,
            "cycloid": self.cycloid_distribution,
            "inverted_cycloid": self.inverted_cycloid_distribution,
            "lame_curve": self.lame_curve_distribution,
            "inverted_lame_curve": self.inverted_lame_curve_distribution,
            "linear": self.linear_distribution,
            "inverted_linear": self.inverted_linear_distribution,
            "even": self.even_distribution,
            #"inverted_even": self.inverted_even_distribution
        }

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        self.END_SWITCH_1_PIN = 23
        self.END_SWITCH_2_PIN = 27
        self.CAMERA_TRIGGER_PIN = 5
        self.ROTATOR_TRIGGER_PIN = 24
        GPIO.setup(self.END_SWITCH_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.END_SWITCH_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.CAMERA_TRIGGER_PIN, GPIO.OUT)
        GPIO.setup(self.ROTATOR_TRIGGER_PIN, GPIO.OUT)
        self.kit = adafruit_motorkit.MotorKit()

    def create_widgets(self):
        self.length_var = tk.IntVar(value=1750)
        self.num_photos_var = tk.IntVar(value=1000)
        self.interval_var = tk.DoubleVar(value=8.0)
        self.exposure_pause_var = tk.DoubleVar(value=2.0)
        self.distribution_var = tk.StringVar(value="catenary")

        ttk.Label(self.root, text="Length:").grid(row=0, column=0, pady=10)
        ttk.Entry(self.root, textvariable=self.length_var).grid(row=0, column=1, pady=10)

        ttk.Label(self.root, text="Number of Photos:").grid(row=1, column=0, pady=10)
        ttk.Entry(self.root, textvariable=self.num_photos_var).grid(row=1, column=1, pady=10)

        ttk.Label(self.root, text="Interval (s):").grid(row=2, column=0, pady=10)
        ttk.Entry(self.root, textvariable=self.interval_var).grid(row=2, column=1, pady=10)

        ttk.Label(self.root, text="Exposure Pause (s):").grid(row=3, column=0, pady=10)
        ttk.Entry(self.root, textvariable=self.exposure_pause_var).grid(row=3, column=1, pady=10)

        ttk.Label(self.root, text="Select Direction:").grid(row=5, column=0, pady=10)
        left_radio = ttk.Radiobutton(self.root, text="Left", variable=self.direction_var, value="left")
        left_radio.grid(row=5, column=1)
        right_radio = ttk.Radiobutton(self.root, text="Right", variable=self.direction_var, value="right")
        right_radio.grid(row=5, column=2)

        ttk.Label(self.root, text="Select Movement Curve:").grid(row=4, column=0, pady=10)
        distribution_combobox = ttk.Combobox(self.root, values=[
            "catenary", "inverted_catenary",
            "gaussian", "inverted_gaussian",
            "ellipsoidal", "inverted_ellipsoidal",
            "parabolic", "inverted_parabolic",
            "cycloid", "inverted_cycloid",
            "lame_curve", "inverted_lame_curve",
            "linear", "inverted_linear",
            "even", #"inverted_even"
        ], textvariable=self.distribution_var)
        distribution_combobox.grid(row=4, column=1, pady=10)

        ttk.Button(self.root, text="Start", command=self.start_camera).grid(row=6, column=1, pady=10)
        ttk.Button(self.root, text="Stop", command=self.stop_camera).grid(row=6, column=2, pady=10)

    def start_camera(self):
        if not self.gantry_running:
            self.gantry_running = True
            self.stop_flag = False
            self.total_photos = self.num_photos_var.get()
            last_trigger_time = time.time() - self.interval_var.get()

            # Start the timelapse in a separate thread
            threading.Thread(target=self.run_timelapse, args=(last_trigger_time,)).start()

    def run_timelapse(self, last_trigger_time):
        try:
            selected_direction = self.direction_var.get()
            if selected_direction == "left":
                self.move_to_right_end()
            elif selected_direction == "right":
                self.move_to_left_end()

            while self.photos_taken < self.total_photos and not self.stop_flag:
                start_time = time.time()

                belt_pitch = 2
                pulley_teeth = 16
                steps_per_revolution = 200

                distance_per_step = (belt_pitch * pulley_teeth) / steps_per_revolution
                length_mm = self.length_var.get()
                total_steps = int(length_mm / distance_per_step)

                selected_distribution = self.distribution_var.get()
                distribution_function = self.distribution_functions[selected_distribution]
                steps_distribution = distribution_function(self.num_photos_var.get(), total_steps)

                # Adjust the steps distribution array if necessary
                if np.sum(steps_distribution) > total_steps:
                    steps_distribution = steps_distribution * (total_steps / np.sum(steps_distribution))

                steps_distribution = np.round(steps_distribution).astype(int)
                scaled_distribution = (steps_distribution / np.sum(steps_distribution)) * total_steps
                scaled_distribution = scaled_distribution.astype(int)  # Convert to integers

                print(f"Steps Distribution Array: {scaled_distribution}")

                intervals = [{'steps': int(steps)} for steps in scaled_distribution]

                for interval in intervals:
                    # Handle remaining time within the interval
                    start_time = time.time()
                    elapsed_time = start_time - last_trigger_time
                    remaining_interval_time = max(0, self.interval_var.get() - elapsed_time)
                    if remaining_interval_time > 0:
                        time.sleep(remaining_interval_time)

                    # Trigger the camera
                    GPIO.output(self.CAMERA_TRIGGER_PIN, GPIO.HIGH)
                    time.sleep(1)  # 1 second low signal
                    GPIO.output(self.CAMERA_TRIGGER_PIN, GPIO.LOW)
                    time.sleep(1)  # 1 second high signal
                    GPIO.output(self.CAMERA_TRIGGER_PIN, GPIO.HIGH)

                    # Pause for exposure pause time
                    time.sleep(self.exposure_pause_var.get())

                    # Move the gantry after taking the photo
                    step_direction = -1 if self.direction_var.get() == 'left' else 1
                    for _ in range(interval['steps']):
                        self.kit.stepper1.onestep(direction=step_direction)
                        time.sleep(0.01 * self.factor)

                    # Trigger the rotator
                    GPIO.output(self.ROTATOR_TRIGGER_PIN, GPIO.HIGH)
                    time.sleep(1)  # 1 second high signal
                    GPIO.output(self.ROTATOR_TRIGGER_PIN, GPIO.LOW)

                    time.sleep(self.MOVEMENT_BUFFER)

                    self.photos_taken += interval['steps']
                    self.photos_remaining = self.total_photos - self.photos_taken

                    if self.stop_flag:
                        print("Stop button pressed. Stopping program.")
                        break

                    # Handle the close event during timelapse
                    if not self.root.winfo_exists():
                        self.stop_flag = True
                        break

        except Exception as e:
            print(f"An error occurred: {e}")
            self.cleanup_and_exit()

        self.cleanup_and_exit()

    def stop_camera(self):
        self.stop_flag = True

    def cleanup_and_exit(self):
        self.gantry_running = False
        GPIO.cleanup()
        self.root.destroy()

    def move_to_left_end(self):
        print("Moving to left end...")
        while GPIO.input(self.END_SWITCH_1_PIN) == GPIO.HIGH:
            self.kit.stepper1.onestep(direction=-1)  # Move towards left end
            time.sleep(0.01 * self.factor)

        # Stop the motor
        self.kit.stepper1.release()

        print("Left end switch triggered.")

        # Move back 20 steps after hitting the left end
        for _ in range(20):
            self.kit.stepper1.onestep(direction=1)
            time.sleep(0.01 * self.factor)

        print("Gantry position initialized.")

    def move_to_right_end(self):
        print("Moving to right end...")
        while GPIO.input(self.END_SWITCH_2_PIN) == GPIO.HIGH:
            self.kit.stepper1.onestep(direction=1)  # Move towards right end
            time.sleep(0.01 * self.factor)

        # Stop the motor
        self.kit.stepper1.release()

        print("Right end switch triggered.")

        # Move back 20 steps after hitting the right end
        for _ in range(20):
            self.kit.stepper1.onestep(direction=-1)
            time.sleep(0.01 * self.factor)

        print("Gantry position initialized.")

    def catenary_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        catenary_curve = max_steps * (np.cosh(x) - 1)
        return self.invert_distribution(catenary_curve)

    def inverted_catenary_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        catenary_curve = max_steps * (np.cosh(x) - 1)
        inverted_curve = max_steps - catenary_curve + 1  # Inversion formula
        return inverted_curve.astype(int)

    def gaussian_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        gaussian_curve = max_steps * np.exp(-0.5 * x**2)
        normalized_curve = (gaussian_curve - np.min(gaussian_curve))
        normalized_curve = (normalized_curve / (np.max(normalized_curve) - np.min(normalized_curve))) * max_steps
        return self.invert_distribution(normalized_curve)

    def inverted_gaussian_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        gaussian_curve = max_steps * np.exp(-0.5 * x**2)
        inverted_curve = max_steps - gaussian_curve + np.min(gaussian_curve)  # Inversion formula
        return inverted_curve.astype(int)

    def ellipsoidal_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        ellipsoidal_curve = max_steps * np.sqrt(1 - x**2)
        return self.invert_distribution(ellipsoidal_curve)

    def inverted_ellipsoidal_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        ellipsoidal_curve = max_steps * np.sqrt(1 - x**2)
        inverted_curve = max_steps - ellipsoidal_curve + np.min(ellipsoidal_curve)  # Inversion formula
        return inverted_curve.astype(int)

    def parabolic_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        parabolic_curve = max_steps * (1 - x**2)
        return self.invert_distribution(parabolic_curve)

    def inverted_parabolic_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        parabolic_curve = max_steps * (1 - x**2)
        inverted_curve = max_steps - parabolic_curve + 1  # Inversion formula
        return inverted_curve.astype(int)

    def cycloid_distribution(self, total_intervals, max_steps):
        x = np.linspace(-np.pi, np.pi, total_intervals)
        cycloid_curve = max_steps * (1 - np.cos(x))
        return self.invert_distribution(cycloid_curve)

    def inverted_cycloid_distribution(self, total_intervals, max_steps):
        x = np.linspace(-np.pi, np.pi, total_intervals)
        cycloid_curve = max_steps * (1 - np.cos(x))
        inverted_curve = np.flip(cycloid_curve)  # Inversion
        return inverted_curve.astype(int)

    def lame_curve_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        lame_curve = max_steps * np.abs(np.tan(x))
        return lame_curve.astype(int)

    def inverted_lame_curve_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        lame_curve = max_steps * np.abs(np.tan(x))
        inverted_curve = max_steps - lame_curve + 1  # Inversion formula
        inverted_curve[inverted_curve < 0] = 0  # Clip negative values to zero
        return inverted_curve.astype(int)

    def linear_distribution(self, total_intervals, max_steps):
        linear_curve = np.linspace(0, max_steps, total_intervals)
        return linear_curve.astype(int)

    def inverted_linear_distribution(self, total_intervals, max_steps):
        linear_curve = np.linspace(0, max_steps, total_intervals)
        inverted_curve = max_steps - linear_curve + 1  # Inversion formula
        return inverted_curve.astype(int)

    def even_distribution(self, total_intervals, max_steps):
        return np.ones(total_intervals) * max_steps

    def inverted_even_distribution(self, total_intervals, max_steps):
        return np.zeros(total_intervals)  # All zeros for inverted even distribution

    def invert_distribution(self, distribution):
        return np.flip(distribution)

if __name__ == "__main__":
    root = tk.Tk()

    # Set DPI scaling factor to scale up interface
    root.tk.call('tk', 'scaling', 4.0)

    root.title("PiSlider Camera Control")
    app = PiSliderApp(root)

    # Bind closing event to stop_camera method
    root.protocol("WM_DELETE_WINDOW", app.stop_camera)

    root.mainloop()
