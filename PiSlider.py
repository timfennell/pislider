import threading
import time
import tkinter
from tkinter import ttk
import numpy as np
import RPi.GPIO as GPIO
from PIL import Image, ImageTk
import os
import math

class PiSliderApp:
    def __init__(self, root):
        self.root = root
        self.root.tk.call('tk', 'scaling', 4.0)
        self.root.title("PiSlider Camera Control")
        self.root.protocol("WM_DELETE_WINDOW", self.stop_camera)

        self.image_dir = "/home/tim/Desktop/PiSlider"

        # --- Hardware Settings ---
        self.END_SWITCH_1_PIN = 24
        self.END_SWITCH_2_PIN = 18
        self.CAMERA_TRIGGER_PIN = 4
        self.slider_dir_pin = 17
        self.slider_step_pin = 27
        self.rotation_dir_pin = 5
        self.rotation_step_pin = 6

        # --- Operational Variables ---
        self.gantry_running = False
        self.stop_flag = False
        self.factor = 1.0
        self.photos_taken = 0
        self.MOVEMENT_BUFFER = 2

        # --- User-Selected Direction ---
        self.user_direction = "right"
        self.rotation_direction = GPIO.HIGH

        # --- Minimum Slider Speed Setting ---
        self.minimum_slider_speed_var = tkinter.IntVar(value=0)
        self.minimum_slider_speed_steps = 0

        # --- GUI Variables ---
        self.direction_var = tkinter.StringVar(value='right')
        self.length_var = tkinter.IntVar(value=1750)
        self.distribution_var = tkinter.StringVar(value="even")
        self.rotation_angle_var = tkinter.DoubleVar(value=180.0)
        self.rotation_direction_var = tkinter.StringVar(value="CW")
        self.rotation_distribution_var = tkinter.StringVar(value="even")
        self.num_photos_var = tkinter.IntVar(value=1000)
        self.interval_var = tkinter.DoubleVar(value=8.0)
        self.exposure_pause_var = tkinter.DoubleVar(value=2.0)

        # Dictionary of available distribution functions
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
            "object_tracking": self.object_tracking_rotation
        }

        self.setup_gpio()
        self.create_widgets()
        self.root.mainloop()

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.END_SWITCH_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.END_SWITCH_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.setup(self.CAMERA_TRIGGER_PIN, GPIO.OUT, initial=GPIO.HIGH)

        GPIO.setup(self.slider_dir_pin, GPIO.OUT)
        GPIO.setup(self.slider_step_pin, GPIO.OUT)
        GPIO.setup(self.rotation_dir_pin, GPIO.OUT)
        GPIO.setup(self.rotation_step_pin, GPIO.OUT)

    def create_widgets(self):
        # Style for wider dropdown arrow button
        style = ttk.Style()
        style.configure("TCombobox", arrowsize=12)
        style.map("TCombobox",
                  foreground=[('readonly', 'black'), ('!disabled', 'black')],
                  background=[('readonly', 'white'), ('!disabled', 'white')],
                  fieldbackground=[('readonly', 'white'), ('!disabled', 'white')],
                  arrowcolor=[('readonly', 'black'), ('!disabled', 'black')])

        # Gantry Settings
        ttk.Label(self.root, text="Length:").grid(row=0, column=0, pady=10)
        ttk.Entry(self.root, textvariable=self.length_var).grid(row=0, column=1, pady=10)

        ttk.Label(self.root, text="Select Direction:").grid(row=1, column=0, pady=10)
        left_radio = ttk.Radiobutton(self.root, text="Left", variable=self.direction_var, value="left",
                                     command=self.update_user_direction)
        left_radio.grid(row=1, column=1)
        right_radio = ttk.Radiobutton(self.root, text="Right", variable=self.direction_var, value="right",
                                      command=self.update_user_direction)
        right_radio.grid(row=1, column=2)

        ttk.Label(self.root, text="Select Movement Curve:").grid(row=2, column=0, pady=10)
        distribution_combobox = ttk.Combobox(self.root, values=[
            "catenary", "inverted_catenary",
            "gaussian", "inverted_gaussian",
            "ellipsoidal", "inverted_ellipsoidal",
            "parabolic", "inverted_parabolic",
            "cycloid", "inverted_cycloid",
            "lame_curve", "inverted_lame_curve",
            "linear", "inverted_linear",
            "even",
        ], textvariable=self.distribution_var, style="TCombobox")
        distribution_combobox.grid(row=2, column=1, pady=10)
        distribution_combobox.bind("<<ComboboxSelected>>", self.update_curve_image)

        # Rotation Settings
        ttk.Label(self.root, text="Rotation Angle (°):").grid(row=3, column=0, pady=10)
        ttk.Entry(self.root, textvariable=self.rotation_angle_var).grid(row=3, column=1, pady=10)

        ttk.Label(self.root, text="Rotation Direction:").grid(row=4, column=0, pady=10)
        cw_radio = ttk.Radiobutton(self.root, text="CCW", variable=self.rotation_direction_var, value="CW",
                                   command=self.update_rotation_direction)
        cw_radio.grid(row=4, column=1)
        ccw_radio = ttk.Radiobutton(self.root, text="CW", variable=self.rotation_direction_var, value="CCW",
                                    command=self.update_rotation_direction)
        ccw_radio.grid(row=4, column=2)

        ttk.Label(self.root, text="Rotation Movement Curve:").grid(row=5, column=0, pady=10)
        rotation_distribution_combobox = ttk.Combobox(self.root, values=[
            "catenary", "inverted_catenary",
            "gaussian", "inverted_gaussian",
            "ellipsoidal", "inverted_ellipsoidal",
            "parabolic", "inverted_parabolic",
            "cycloid", "inverted_cycloid",
            "lame_curve", "inverted_lame_curve",
            "linear", "inverted_linear",
            "even",
            "object_tracking"
        ], textvariable=self.rotation_distribution_var, style="TCombobox")
        rotation_distribution_combobox.grid(row=5, column=1, pady=10)

        # Timelapse Settings
        ttk.Label(self.root, text="Number of Photos:").grid(row=6, column=0, pady=10)
        ttk.Entry(self.root, textvariable=self.num_photos_var).grid(row=6, column=1, pady=10)

        ttk.Label(self.root, text="Interval (s):").grid(row=7, column=0, pady=10)
        ttk.Entry(self.root, textvariable=self.interval_var).grid(row=7, column=1, pady=10)

        ttk.Label(self.root, text="Exposure Pause (s):").grid(row=8, column=0, pady=10)
        ttk.Entry(self.root, textvariable=self.exposure_pause_var).grid(row=8, column=1, pady=10)

        # Minimum Slider Speed Setting (NOW IN STEPS)
        ttk.Label(self.root, text="Minimum Slider Speed (steps/interval):").grid(row=10, column=0, pady=10)
        self.minimum_slider_speed_entry = ttk.Entry(self.root, textvariable=self.minimum_slider_speed_var)
        self.minimum_slider_speed_entry.grid(row=10, column=1, pady=10)

        # Action Buttons
        ttk.Button(self.root, text="Start", command=self.start_camera).grid(row=12, column=1, pady=10)
        ttk.Button(self.root, text="Stop", command=self.stop_camera).grid(row=13, column=1, pady=10)

        # Curve Image Display
        self.curve_image_label = tkinter.Label(self.root)
        self.curve_image_label.grid(row=0, column=3, rowspan=10, padx=10, pady=10, sticky="ne")
        self.update_curve_image()

    def update_curve_image(self, event=None):
        selected_distribution = self.distribution_var.get()
        image_path = os.path.join(self.image_dir, f"{selected_distribution}.png")
        try:
            image = Image.open(image_path)
            image = image.resize((150, 150))
            photo = ImageTk.PhotoImage(image)
            self.curve_image_label.config(image=photo)
            self.curve_image_label.image = photo
        except FileNotFoundError:
            print(f"Image file {image_path} not found.")

    def update_user_direction(self):
        self.user_direction = self.direction_var.get()

    def update_rotation_direction(self):
        self.rotation_direction = GPIO.HIGH if self.rotation_direction_var.get() == "CW" else GPIO.LOW

    def start_camera(self):
        if not self.gantry_running:
            self.gantry_running = True
            self.stop_flag = False
            self.photos_taken = 0
            self.total_photos = self.num_photos_var.get()
            last_trigger_time = time.time() - self.interval_var.get()

            self.minimum_slider_speed_steps = self.minimum_slider_speed_var.get()

            self.calculate_distribution_arrays()

            threading.Thread(target=self.run_timelapse, args=(last_trigger_time,)).start()

    def calculate_distribution_arrays(self):
        """Calculates the step distribution arrays for both slider and rotation."""
        belt_pitch = 2
        pulley_teeth = 16
        steps_per_revolution_slider = 200
        steps_per_revolution_rotation = 200 * 30

        distance_per_step = (belt_pitch * pulley_teeth) / steps_per_revolution_slider

        # --- Slider Movement Calculation---
        length_mm = self.length_var.get()
        self.total_steps = int(length_mm / distance_per_step)

        curve_total_steps = self.total_steps

        selected_distribution = self.distribution_var.get()
        distribution_function = self.distribution_functions[selected_distribution]

        if selected_distribution == "even":
            self.steps_distribution = distribution_function(self.num_photos_var.get(), curve_total_steps)
        else:
            self.steps_distribution = distribution_function(self.num_photos_var.get(), curve_total_steps)
            self.steps_distribution = self.translate_array(self.steps_distribution, 0)

            for i in range(self.num_photos_var.get()):
                self.steps_distribution[i] += self.minimum_slider_speed_steps

            # --- Scaling the Slider Array (NOW BEFORE INTEGER CONVERSION) --- 
            self.steps_distribution = self.steps_distribution.astype(float) # Convert to float first
            self.steps_distribution = self.scale_array(self.steps_distribution, self.total_steps)
            self.steps_distribution = self.steps_distribution.astype(int) # Convert to integer AFTER scaling

        # --- Rotation Movement Calculation ---
        selected_rotation_distribution = self.rotation_distribution_var.get()
        rotation_distribution_function = self.distribution_functions.get(
            selected_rotation_distribution, self.even_distribution
        )

        if selected_rotation_distribution == "object_tracking":
            # Object Tracking Rotation Calculation
            self.rotation_steps_distribution = self.object_tracking_rotation(
                self.num_photos_var.get(), self.total_steps, self.rotation_angle_var.get()
            )
        else:
            # Calculations for other distribution curves (remain the same)
            rotation_angle = self.rotation_angle_var.get()
            total_rotation_steps = int(steps_per_revolution_rotation * (rotation_angle / 360))

            if selected_rotation_distribution == "even":
                self.rotation_steps_distribution = rotation_distribution_function(
                    self.num_photos_var.get(), total_rotation_steps
                )
            else:
                self.rotation_steps_distribution = rotation_distribution_function(
                    self.num_photos_var.get(), total_rotation_steps
                )
                self.rotation_steps_distribution = self.translate_array(self.rotation_steps_distribution, 0)
                self.rotation_steps_distribution = self.scale_array(
                    self.rotation_steps_distribution, total_rotation_steps
                )

        # --- Print the Arrays ---
        print(f"Slider Steps Distribution: {self.steps_distribution}")
        print(f"Rotation Steps Distribution: {self.rotation_steps_distribution}")

        # --- Combine Slider and Rotation Intervals ---
        self.combined_intervals = []
        for i in range(self.num_photos_var.get()):
            self.combined_intervals.append({
                'slider_steps': int(self.steps_distribution[i]),
                'rotation_steps': int(self.rotation_steps_distribution[i])
            })

    def run_timelapse(self, last_trigger_time):
        try:
            # --- Initialize to the Correct End and Move 20 Steps ---
            # ONLY move to the correct end ONCE
            if self.user_direction == "left":
                self.move_to_right_end()
                time.sleep(self.MOVEMENT_BUFFER)
                self.move_steps(20, GPIO.HIGH)
            elif self.user_direction == "right":
                self.move_to_left_end()
                time.sleep(self.MOVEMENT_BUFFER)
                self.move_steps(20, GPIO.LOW)

            # --- Timelapse Capture Loop ---
            for interval in self.combined_intervals:
                if GPIO.input(self.END_SWITCH_1_PIN) == GPIO.LOW or GPIO.input(self.END_SWITCH_2_PIN) == GPIO.LOW:
                    print("Either end switch triggered. Stopping program.")
                    self.stop_camera()
                    return

                # --- Image Capture ---
                GPIO.output(self.CAMERA_TRIGGER_PIN, GPIO.LOW)
                time.sleep(1)
                GPIO.output(self.CAMERA_TRIGGER_PIN, GPIO.HIGH)

                time.sleep(self.exposure_pause_var.get())

                # --- Slider Movement ---
                self.set_slider_step(interval['slider_steps'],
                                     GPIO.HIGH if self.user_direction == "left" else GPIO.LOW)

                # --- Rotation Movement ---
                for _ in range(interval['rotation_steps']):
                    GPIO.output(self.rotation_dir_pin, self.rotation_direction)
                    GPIO.output(self.rotation_step_pin, GPIO.HIGH)
                    time.sleep(0.001)
                    GPIO.output(self.rotation_step_pin, GPIO.LOW)
                    time.sleep(0.001)

                # --- Wait for the next interval ---
                current_time = time.time()
                wait_time = self.interval_var.get() - (current_time - last_trigger_time)
                if wait_time > 0:
                    time.sleep(wait_time)

                last_trigger_time = time.time()

                self.photos_taken += 1
                self.photos_remaining = self.total_photos - self.photos_taken

                if self.stop_flag:
                    print("Stop button pressed. Stopping program.")
                    break

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
        print("Moving to left end for initialization...")
        GPIO.output(self.slider_dir_pin, GPIO.HIGH)

        while True:
            GPIO.output(self.slider_step_pin, GPIO.HIGH)
            time.sleep(0.005)
            GPIO.output(self.slider_step_pin, GPIO.LOW)
            time.sleep(0.005)

            if GPIO.input(self.END_SWITCH_1_PIN) == GPIO.LOW:
                print("End switch triggered.")
                time.sleep(0.1)
                if GPIO.input(self.END_SWITCH_1_PIN) == GPIO.LOW:
                    break

        self.slider_direction = -1

        self.move_steps(20, GPIO.LOW)
        time.sleep(0.1)
        self.move_steps(20, GPIO.LOW)

        print("Gantry position initialized.")

    def move_to_right_end(self):
        print("Moving to right end for initialization...")
        GPIO.output(self.slider_dir_pin, GPIO.LOW)

        while True:
            GPIO.output(self.slider_step_pin, GPIO.HIGH)
            time.sleep(0.005)
            GPIO.output(self.slider_step_pin, GPIO.LOW)
            time.sleep(0.005)

            if GPIO.input(self.END_SWITCH_2_PIN) == GPIO.LOW:
                print("End switch triggered.")
                time.sleep(0.1)
                if GPIO.input(self.END_SWITCH_2_PIN) == GPIO.LOW:
                    break

        self.slider_direction = 1

        self.move_steps(20, GPIO.HIGH)
        time.sleep(0.1)
        self.move_steps(20, GPIO.HIGH)

        print("Gantry position initialized.")

    def move_steps(self, num_steps, direction):
        for _ in range(num_steps):
            GPIO.output(self.slider_dir_pin, direction)
            GPIO.output(self.slider_step_pin, GPIO.HIGH)
            time.sleep(0.005)
            GPIO.output(self.slider_step_pin, GPIO.LOW)
            time.sleep(0.005)

    # --- Slider Distribution Functions ---
    def catenary_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        catenary_curve = max_steps * (np.cosh(x) - 1)
        catenary_curve = catenary_curve / np.max(catenary_curve) * max_steps
        return catenary_curve.astype(float)  # Return as float

    def inverted_catenary_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        catenary_curve = max_steps * (np.cosh(x) - 1)
        catenary_curve = catenary_curve / np.max(catenary_curve) * max_steps
        inverted_curve = max_steps - catenary_curve
        return inverted_curve.astype(float)  # Return as float

    def gaussian_distribution(self, total_intervals, max_steps):
        x = np.linspace(-3, 3, total_intervals)
        gaussian_curve = max_steps * np.exp(-x ** 2)
        gaussian_curve = gaussian_curve / np.max(gaussian_curve) * max_steps
        return gaussian_curve.astype(float)  # Return as float

    def inverted_gaussian_distribution(self, total_intervals, max_steps):
        x = np.linspace(-3, 3, total_intervals)
        gaussian_curve = max_steps * np.exp(-x ** 2)
        gaussian_curve = gaussian_curve / np.max(gaussian_curve) * max_steps
        inverted_curve = max_steps - gaussian_curve
        return inverted_curve.astype(float)  # Return as float

    def ellipsoidal_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        ellipsoidal_curve = max_steps * (1 - x ** 2) ** 0.5
        ellipsoidal_curve = ellipsoidal_curve / np.max(ellipsoidal_curve) * max_steps
        return ellipsoidal_curve.astype(float)  # Return as float

    def inverted_ellipsoidal_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        ellipsoidal_curve = max_steps * (1 - x ** 2) ** 0.5
        ellipsoidal_curve = ellipsoidal_curve / np.max(ellipsoidal_curve) * max_steps
        inverted_curve = max_steps - ellipsoidal_curve
        return inverted_curve.astype(float)  # Return as float

    def parabolic_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        parabolic_curve = max_steps * (1 - x ** 2)
        parabolic_curve = parabolic_curve / np.max(parabolic_curve) * max_steps
        return parabolic_curve.astype(float)  # Return as float

    def inverted_parabolic_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        parabolic_curve = max_steps * (1 - x ** 2)
        parabolic_curve = parabolic_curve / np.max(parabolic_curve) * max_steps
        inverted_curve = max_steps - parabolic_curve
        return inverted_curve.astype(float)  # Return as float

    def cycloid_distribution(self, total_intervals, max_steps):
        x = np.linspace(-np.pi, np.pi, total_intervals)
        cycloid_curve = max_steps * (1 - np.cos(x)) / 2
        return cycloid_curve.astype(float)  # Return as float

    def inverted_cycloid_distribution(self, total_intervals, max_steps):
        x = np.linspace(-np.pi, np.pi, total_intervals)
        cycloid_curve = max_steps * (1 - np.cos(x)) / 2
        inverted_curve = max_steps - cycloid_curve
        return inverted_curve.astype(float)  # Return as float

    def lame_curve_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        lame_curve = max_steps * (1 - x ** 10) ** 0.1
        lame_curve = lame_curve / np.max(lame_curve) * max_steps
        return lame_curve.astype(float)  # Return as float

    def inverted_lame_curve_distribution(self, total_intervals, max_steps):
        x = np.linspace(-1, 1, total_intervals)
        lame_curve = max_steps * (1 - x ** 10) ** 0.1
        lame_curve = lame_curve / np.max(lame_curve) * max_steps
        inverted_curve = max_steps - lame_curve
        return inverted_curve.astype(float)  # Return as float

    def linear_distribution(self, total_intervals, max_steps):
        linear_curve = np.linspace(0, max_steps, total_intervals)
        return linear_curve.astype(float)  # Return as float

    def inverted_linear_distribution(self, total_intervals, max_steps):
        linear_curve = np.linspace(0, max_steps, total_intervals)
        inverted_curve = max_steps - linear_curve
        return inverted_curve.astype(float)  # Return as float

    def even_distribution(self, total_intervals, max_steps):
        even_steps = max(1, max_steps // total_intervals)
        even_curve = np.full(total_intervals, even_steps)
        return even_curve.astype(float)  # Return as float

    def object_tracking_rotation(self, total_intervals, total_slider_steps, total_rotation_angle):
        """Calculates rotation steps to keep the camera pointed at a stationary object
           using the arctangent function for accurate angle calculation.

        Assumptions:
        - The camera starts pointed directly at the object.
        - The object is positioned above the middle of the slider's path.
        - Rotation direction (CW or CCW) is handled using self.rotation_direction in run_timelapse.
        - The output array contains only positive integers representing rotation steps.
        """

        rotation_steps_distribution = []
        steps_per_revolution_rotation = 200 * 30  # Adjust for your motor
        total_rotation_steps = (total_rotation_angle / 360) * steps_per_revolution_rotation  # Keep as float

        # Calculate cumulative slider movement at each interval
        cumulative_slider_movement = np.cumsum(self.steps_distribution)

        # Initialize previous angle
        previous_angle = 0

        for i in range(total_intervals):
            # Calculate distance from center for this interval
            if self.user_direction == "left":
                distance_from_center = (cumulative_slider_movement[i] - self.total_steps/2)
            else:
                distance_from_center = (self.total_steps/2 - cumulative_slider_movement[i]) 

            # Use arctan to calculate target angle in radians
            target_angle_rad = math.atan(distance_from_center / (self.total_steps / 2))  

            # Convert radians to degrees 
            target_angle = math.degrees(target_angle_rad) 

            # Set angle sign based on user's direction
            if self.user_direction == "left":
                target_angle *= -1 

            # Calculate incremental angle change from the previous position
            incremental_angle = target_angle - previous_angle

            # Calculate rotation steps (float)
            rotation_steps = (incremental_angle / 360) * total_rotation_steps 
            
            # Convert rotation steps to positive and append 
            rotation_steps_distribution.append(abs(rotation_steps)) 

            # Update previous angle 
            previous_angle = target_angle 

        #  Copy the second value to the first to make it symmetrical
        rotation_steps_distribution[0] = rotation_steps_distribution[1]  

        # Scale the rotation steps distribution 
        rotation_steps_distribution = self.scale_array(rotation_steps_distribution, total_rotation_steps)

        # Convert to integers only AFTER scaling
        return np.round(rotation_steps_distribution).astype(int)

    def scale_array(self, array, max_steps):
        """Scales the array so the sum matches the specified maximum steps."""
        array = np.array(array)
        scaling_factor = max_steps / np.sum(array)
        array = array.astype(float)  # Convert to float before scaling
        array *= scaling_factor
        return array.astype(int)  # Convert to int AFTER scaling

    def translate_array(self, array, minimum_steps):
        """Translates the array to start at the specified minimum steps."""
        array = array - np.min(array)
        array = array + minimum_steps
        return array.astype(int)

    def set_slider_step(self, num_steps, direction):
        for _ in range(num_steps):
            GPIO.output(self.slider_dir_pin, direction)
            GPIO.output(self.slider_step_pin, GPIO.HIGH)
            time.sleep(0.005)
            GPIO.output(self.slider_step_pin, GPIO.LOW)
            time.sleep(0.005)

if __name__ == "__main__":
    root = tkinter.Tk()
    app = PiSliderApp(root)
