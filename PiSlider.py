import time
import tkinter as tk
from tkinter import ttk
import numpy as np
from adafruit_motorkit import MotorKit
import RPi.GPIO as GPIO

class PiSlider:
    def __init__(self, root):
        GPIO.setmode(GPIO.BCM)

        self.root = root
        self.root.title("PiSlider")

        self.END_SWITCH_1_PIN = 23
        self.END_SWITCH_2_PIN = 27
        self.CAMERA_TRIGGER_PIN = 5
        self.ROTATOR_TRIGGER_PIN = 24

        GPIO.setup(self.END_SWITCH_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.END_SWITCH_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.CAMERA_TRIGGER_PIN, GPIO.OUT)
        GPIO.setup(self.ROTATOR_TRIGGER_PIN, GPIO.OUT)

        self.kit = MotorKit()

        self.length_var = tk.IntVar(value=1760)
        self.num_photos_var = tk.IntVar(value=1000)
        self.interval_var = tk.DoubleVar(value=8.0)
        self.exposure_pause_var = tk.DoubleVar(value=2.0)
        self.direction_var = tk.StringVar(value="left")
        self.distribution_var = tk.StringVar(value="even")

        self.PITCH_OF_BELT_MM = 2.0
        self.TEETH_ON_PULLEY = 16

        ttk.Label(root, text="Length:").grid(row=0, column=0, pady=10)
        ttk.Entry(root, textvariable=self.length_var).grid(row=0, column=1, pady=10)

        ttk.Label(root, text="Number of Photos:").grid(row=1, column=0, pady=10)
        ttk.Entry(root, textvariable=self.num_photos_var).grid(row=1, column=1, pady=10)

        ttk.Label(root, text="Interval (s):").grid(row=2, column=0, pady=10)
        ttk.Entry(root, textvariable=self.interval_var).grid(row=2, column=1, pady=10)

        ttk.Label(root, text="Exposure Pause (s):").grid(row=3, column=0, pady=10)
        ttk.Entry(root, textvariable=self.exposure_pause_var).grid(row=3, column=1, pady=10)

        ttk.Label(root, text="Direction Select:").grid(row=4, column=0, pady=10)
        direction_combobox = ttk.Combobox(root, values=["left", "right"], textvariable=self.direction_var)
        direction_combobox.grid(row=4, column=1, pady=10)

        ttk.Label(root, text="Select Movement Curve:").grid(row=5, column=0, pady=10)
        distribution_combobox = ttk.Combobox(root, values=["catenary", "gaussian", "ellipsoidal", "parabolic", "cycloid", "lame_curve", "linear", "even"], textvariable=self.distribution_var)
        distribution_combobox.grid(row=5, column=1, pady=10)

        ttk.Button(root, text="Invert Distribution", command=self.invert_distribution).grid(row=6, column=0, pady=10)
        ttk.Button(root, text="Start", command=self.start_camera).grid(row=6, column=1, pady=10)

        self.LOOP_INTERVAL = self.interval_var.get()  # Updated to use GUI interval value
        self.EXPOSURE_PAUSE = 1
        self.MOVEMENT_BUFFER = 1
        self.STEPS_PER_REVOLUTION = 200

        self.kit = MotorKit()

        self.factor = 0.25
        self.gantry_running = False

        self.total_photos = self.num_photos_var.get()
        self.photos_taken = 0
        self.photos_remaining = self.total_photos

        self.stop_flag = False
        root.after(100, self.check_stop_flag)

        root.bind("<KeyPress>", self.key_press_callback)

        root.mainloop()

    def start_camera(self):
        if not self.gantry_running:
            self.gantry_running = True
            self.stop_flag = False

            try:
                self.initialize_gantry_position()  # Initialize gantry position

                while self.photos_taken < self.total_photos and not self.stop_flag:
                    start_time = time.time()

                    distance_per_rotation = self.PITCH_OF_BELT_MM * self.TEETH_ON_PULLEY
                    total_steps = int(self.length_var.get() / distance_per_rotation) * self.STEPS_PER_REVOLUTION

                    selected_distribution = self.distribution_var.get()

                    if selected_distribution == "catenary":
                        steps_distribution = self.catenary_distribution(self.num_photos_var.get(), total_steps)
                    elif selected_distribution == "gaussian":
                        steps_distribution = self.gaussian_distribution(self.num_photos_var.get(), total_steps)
                    elif selected_distribution == "ellipsoidal":
                        steps_distribution = self.ellipsoidal_distribution(self.num_photos_var.get(), total_steps)
                    elif selected_distribution == "parabolic":
                        steps_distribution = self.parabolic_distribution(self.num_photos_var.get(), total_steps)
                    elif selected_distribution == "cycloid":
                        steps_distribution = self.cycloid_distribution(self.num_photos_var.get(), total_steps)
                    elif selected_distribution == "lame_curve":
                        steps_distribution = self.lame_curve_distribution(self.num_photos_var.get(), total_steps)
                    elif selected_distribution == "linear":
                        steps_distribution = self.linear_distribution(self.num_photos_var.get(), total_steps)
                    elif selected_distribution == "even":
                        steps_distribution = self.even_distribution(self.num_photos_var.get(), total_steps)
                    else:
                        print("Invalid distribution type. Exiting.")
                        self.cleanup_and_exit()

                    steps_distribution = steps_distribution * (total_steps / np.sum(steps_distribution))

                    intervals = [{'steps': int(steps)} for steps in steps_distribution]

                    for i, interval in enumerate(intervals):
                        print(f"Interval {i + 1} of {len(intervals)}")

                        GPIO.output(self.CAMERA_TRIGGER_PIN, GPIO.LOW)
                        time.sleep(0.5)
                        GPIO.output(self.CAMERA_TRIGGER_PIN, GPIO.HIGH)
                        time.sleep(self.EXPOSURE_PAUSE)

                        for _ in range(interval['steps']):
                            if GPIO.input(self.END_SWITCH_1_PIN) == GPIO.LOW or GPIO.input(self.END_SWITCH_2_PIN) == GPIO.LOW:
                                print("End switch triggered. Reversing away from the end switch.")
                                for _ in range(40):
                                    step_direction = -1 if self.direction_var.get() == 'right' else 1
                                    self.kit.stepper1.onestep(direction=step_direction)
                                    time.sleep(0.01 * self.factor)
                                print("Stopping program.")
                                self.cleanup_and_exit()

                            step_direction = 1 if self.direction_var.get() == 'right' else -1
                            self.kit.stepper1.onestep(direction=step_direction)
                            time.sleep(0.01 * self.factor)

                        GPIO.output(self.ROTATOR_TRIGGER_PIN, GPIO.HIGH)
                        time.sleep(0.5)
                        GPIO.output(self.ROTATOR_TRIGGER_PIN, GPIO.LOW)

                        time.sleep(self.MOVEMENT_BUFFER)

                        elapsed_time = time.time() - start_time
                        remaining_time = max(0, self.LOOP_INTERVAL - elapsed_time)

                        if remaining_time > 0:
                            time.sleep(remaining_time)

                        self.photos_taken += interval['steps']
                        self.photos_remaining = self.total_photos - self.photos_taken

                        if self.stop_flag:
                            print("Stop button pressed. Stopping program.")
                            self.cleanup_and_exit()

                    self.cleanup_and_exit()

            except Exception as e:
                print(f"Error in start_camera: {e}")
                self.cleanup_and_exit()

    def initialize_gantry_position(self):
        try:

            direction = 1 if self.direction_var.get() == 'left' else -1

            # Move until the end switch is triggered
            while GPIO.input(self.END_SWITCH_1_PIN) == GPIO.HIGH and GPIO.input(self.END_SWITCH_2_PIN) == GPIO.HIGH:
                self.kit.stepper1.onestep(direction=direction)
                time.sleep(0.01 * self.factor)

            # Move back by 20 steps
            for _ in range(20):
                self.kit.stepper1.onestep(direction=-direction)
                time.sleep(0.01 * self.factor)

            # Ensure the motor is stopped
            self.kit.stepper1.release()

            # Wait for 5 seconds before starting the main loop
            time.sleep(5)

        except Exception as e:
            print(f"Error in initialize_gantry_position: {e}")
            self.cleanup_and_exit()

    def invert_distribution(self):
        current_distribution = self.distribution_var.get()

        invertible_distributions = ["catenary", "gaussian", "ellipsoidal", "parabolic", "cycloid", "lame_curve", "linear", "even"]

        if current_distribution in invertible_distributions:
            distribution_function = getattr(self, f"{current_distribution}_distribution")
            inverted_distribution = self.invert_array(distribution_function(self.num_photos_var.get(), 100))
            self.distribution_var.set(inverted_distribution)
            print(f"Distribution inverted: {inverted_distribution}")
        else:
            print("Selected distribution cannot be inverted.")

    def invert_array(self, array):
        inverted_array = np.max(array) - array
        return inverted_array.astype(int)

    def stop_camera(self):
        if self.gantry_running:
            self.gantry_running = False
            self.stop_flag = True

    def cleanup_and_exit(self):
        self.gantry_running = False

        try:
            self.kit.stepper1.release()
            GPIO.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")

        self.root.destroy()

    def check_stop_flag(self):
        if self.gantry_running and not self.stop_flag:
            self.root.after(100, self.check_stop_flag)

    def catenary_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        catenary_curve = max_steps * (np.cosh(x) - 1)
        return catenary_curve.astype(int)

    def gaussian_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        gaussian_curve = max_steps * np.exp(-0.5 * x**2)
        normalized_curve = (gaussian_curve - np.min(gaussian_curve))
        normalized_curve = (normalized_curve / (np.max(normalized_curve) - np.min(normalized_curve))) * max_steps
        return normalized_curve.astype(int)

    def ellipsoidal_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        ellipsoidal_curve = max_steps * np.sqrt(1 - x**2)
        return ellipsoidal_curve.astype(int)

    def parabolic_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        parabolic_curve = max_steps * (1 - x**2)
        return parabolic_curve.astype(int)

    def cycloid_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        cycloid_curve = max_steps * (x - 0.5 * np.sin(2 * np.pi * x))
        normalized_curve = (cycloid_curve - np.min(cycloid_curve))
        normalized_curve = (normalized_curve / (np.max(normalized_curve) - np.min(normalized_curve))) * max_steps
        return normalized_curve.astype(int)

    def lame_curve_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        lame_curve = max_steps * x * np.sqrt(1 - x**2)
        normalized_curve = (lame_curve - np.min(lame_curve))
        normalized_curve = (normalized_curve / (np.max(normalized_curve) - np.min(normalized_curve))) * max_steps
        return normalized_curve.astype(int)

    def linear_distribution(self, total_intervals, max_steps):
        linear_curve = np.linspace(0, max_steps, total_intervals)
        return linear_curve.astype(int)

    def even_distribution(self, total_intervals, max_steps):
        return np.ones(total_intervals) * (max_steps / total_intervals)

    def key_press_callback(self, event):
        if event.char == '`':
            self.invert_distribution()

if __name__ == "__main__":
    root = tk.Tk()
    app = PiSlider(root)
