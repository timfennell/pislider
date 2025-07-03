# pislider
Raspberry Pi Controlled Motorized Camera Slider/Rotation timelapse controller with advanced "Holy Grail" capabilities. The current version is designed to run on raspberry pi zero w2 and with a Sony A7III camera.

Project Description & Capabilities

This project is a powerful and flexible camera slider and motion control system designed to run on a Raspberry Pi. It provides a graphical user interface (GUI) built with Tkinter for controlling complex camera movements and shooting sequences for timelapse photography.


Core Features:

Dual-Axis Control: Independently or simultaneously control linear slider movement and 360° panoramic rotation.
Flexible Distribution Curves: Go beyond simple, linear movement. The system offers a wide array of mathematical distribution curves (e.g., Catenary, Gaussian, Parabolic, Cycloid) for both slider and rotation axes. This allows for smooth "ease-in" and "ease-out" effects, creating more dynamic and professional-looking timelapses.
Object Tracking: Automatically calculate the correct rotation angles to keep the camera pointed at a stationary subject as the slider moves, perfect for parallax effects.
Run Preview Mode: Safely test your entire programmed move without taking photos to ensure the motion, speed, and framing are exactly what you want before committing to a long shoot.


Holy Grail Timelapse Mode:

This is the system's most advanced feature, designed to capture seamless day-to-night or night-to-day "Holy Grail" timelapses.
Predictive Exposure Ramping: Based on your latitude, longitude, and the current time, the system automatically calculates the sun's position.
Smooth Setting Adjustments: It then gradually and smoothly adjusts your camera's Aperture, Shutter Speed, and ISO over the entire sequence to perfectly compensate for the changing ambient light.
Flicker-Free Results: By avoiding the sudden jumps in settings that a camera's auto-exposure would make, this mode produces exceptionally smooth, flicker-free results across massive changes in brightness.


Fully Customizable:

All Holy Grail parameters, including the exposure curve, aperture range, and ISO limits, are fully configurable through the GUI.


Camera & Hardware Control:

Dual Triggering Options:

S2 Shutter Cable: A reliable, physical trigger for any camera with an S2-type port.
USB Control: Full camera control via gphoto2 for supported cameras, allowing the app to manage settings directly.
Live Preview: In USB Control mode, you can capture a full-resolution preview image from the camera and display it directly in the app to check focus and composition.
Advanced Motor Control: Utilizes TMC2209 drivers in UART mode, enabling software control over motor current for features like a "Low Power" mode to conserve battery and reduce heat during less demanding horizontal moves.

See the video of my slider on YouTube
https://youtu.be/Z4fMwQC2de0?si=qMtr0yOHDJthevD9

Hardware:

I built my own stepper driver hat for this version of the project based on TMC2209 Stepper Driver Modules and a Proto Breadboard HAT by MakerSpot.
https://www.amazon.ca/dp/B0CJBK66KW?th=1
https://www.amazon.ca/dp/B07B4TRVFD

I complete wiring diagram will be coming soon. I had wanted to design a PCB for the hat, but I need to learn how to use KiCad first.

PiShop.ca

Raspberry Pi Zero 2 W
https://www.pishop.ca/product/raspberry-pi-zero-w/](https://www.pishop.ca/product/raspberry-pi-zero-2-w-with-pre-soldered-headers


Aliexpress

Arca Swiss Plates
2x PU70 Type B
https://www.aliexpress.com/item/1005004095101551.html?spm=a2g0o.order_list.order_list_main.49.684718025XWOeY

Timing Pulley 16 Tooth
16T W6 B5
https://www.aliexpress.com/item/1005003176136280.html?spm=a2g0o.order_list.order_list_main.22.684718025XWOeY

304 Stainless Steel Allen Hexagon Hex Socket Cap Head Screw Bolt and Nut
1x M6 1x M3
https://www.aliexpress.com/item/1005003776689434.html?spm=a2g0o.order_list.order_list_main.53.684718025XWOeY

Black 2040 V- Slot Aluminum Profile Extrusion
2x 1000mm
https://www.aliexpress.com/item/1005003023027840.html?spm=a2g0o.order_list.order_list_main.43.684718025XWOeY

8Pcs 2020 Series Aluminum Profile Straight Line Connector
https://www.aliexpress.com/item/1005004518404319.html?spm=a2g0o.order_list.order_list_main.44.684718025XWOeY

V-Slot Gantry Set 20-80mm Gantry Plate With Wheels
With 6 black wheels
https://www.aliexpress.com/item/1005003983256336.html?spm=a2g0o.order_list.order_list_main.38.684718025XWOeY

NEMA 17 42-Series Stepper Motor Mounting Plate
Black
https://www.aliexpress.com/item/4001250519899.html?spm=a2g0o.order_list.order_list_main.39.684718025XWOeY

X axis V-Slot profile 2040 Y axis synchronous belt Stretch Straighten tensioner
X2 (You must add two to your cart)
https://www.aliexpress.com/item/1005003128683160.html?spm=a2g0o.order_list.order_list_main.31.684718025XWOeY

Belt PU with Steel Core GT2 Belt 2GT Timing Belt Width 6mm
(GT2-6mm)-5Meters
https://www.aliexpress.com/item/1005003262891930.html?spm=a2g0o.order_list.order_list_main.32.684718025XWOeY

20pcs/lot 2020 M3 M4 M5 T Block Square nuts
M5
https://www.aliexpress.com/item/1005003237906405.html?spm=a2g0o.order_list.order_list_main.33.684718025XWOeY

4x 2.5mm 2Pole Male + 4x 2.5mm 2Pole Female
https://www.aliexpress.com/item/1005005341083840.html?spm=a2g0o.order_list.order_list_main.10.684718025XWOeY

High Temperature Blcak Silicone Solid Spacer
Black, 4pcs 18mm
https://www.aliexpress.com/item/1005001823789355.html?spm=a2g0o.order_list.order_list_main.54.69e51802TFKjbD

3D Printer Accessories X/Y/Z Axis End Stop Limit Switch
With Cable, 2PCS
https://www.aliexpress.com/item/1005001834951972.html?spm=a2g0o.order_detail.order_detail_item.7.2fcff19cFD9bqY


Amazon.ca

LeMotech ABS Plastic Junction Box
3.9"x2.7"x2"
https://www.amazon.ca/gp/product/B07C97HXX8/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&th=1

Type-C Fast Charging Trigger Module
https://www.amazon.ca/gp/product/B091GZKML7/ref=ppx_yo_dt_b_asin_title_o07_s00?ie=UTF8&psc=1

UGREEN Mini HDMI Adapter Mini HDMI to HDMI Female Cable
https://www.amazon.ca/gp/product/B00B2HOS08/ref=ppx_yo_dt_b_asin_title_o07_s00?ie=UTF8&th=1

VIXLW 4K HDMI Video Capture Card,
https://www.amazon.ca/gp/product/B0C77P1SM1/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1


Project Dependencies
This document lists all the hardware, system software, and Python libraries required to run the PiSlider project.

1. Hardware
This project is designed to run on a Raspberry Pi and control specific camera slider hardware.
Computer: Raspberry Pi (Model 3B+ or newer recommended for performance).
Motor Drivers: 2x TMC2209 stepper motor drivers configured for UART mode.
Stepper Motors: 2x NEMA 17 stepper motors (or similar) for the slider and rotation axes.
End-Stops: 2x Mechanical or optical end-stop switches.

Camera Trigger:
An S2-type shutter release cable for physical triggering.
A USB cable for "USB Control" mode.
Power Supply: A stable power supply sufficient for the Raspberry Pi and both stepper motors.
2. System-Level Software
These command-line tools must be installed on the Raspberry Pi OS for full functionality. They are used for USB camera control and image processing.
Install them using apt:
Generated bash
sudo apt-get update
sudo apt-get install gphoto2 dcraw -y
Use code with caution.
Bash
gphoto2: The core utility for controlling digital cameras via USB. It is required for all "USB Control" features, including triggering, setting camera parameters, and downloading images.
dcraw: A powerful RAW image decoder. It is used by the "Take Preview" feature to quickly extract a viewable JPEG from the camera's RAW file.
3. Python Libraries
These libraries are required for the Python scripts to run. They can be installed using the Python package manager, pip.
Recommended Installation (using requirements.txt)
The easiest way to install all Python dependencies at once is to use a requirements.txt file.
Create a file named requirements.txt in your project directory.
Copy and paste the following lines into that file:
Generated code
numpy
pyserial
RPi.GPIO
Pillow
astral==1.10.1
pytz
Use code with caution.
Install all of them with a single command:
Generated bash
python3 -m pip install -r requirements.txt
Use code with caution.
Bash
Manual Installation
If you prefer to install them one by one, here are the libraries and their purpose:
numpy: Used for numerical operations, especially for calculating the movement distribution curves.
Generated bash
python3 -m pip install numpy
Use code with caution.
Bash
pyserial: Required for UART serial communication with the TMC2209 motor drivers.
Generated bash
python3 -m pip install pyserial
Use code with caution.
Bash
RPi.GPIO: The standard library for controlling the Raspberry Pi's GPIO pins (for motors, end-stops, and the S2 trigger).
Generated bash
python3 -m pip install RPi.GPIO
Use code with caution.
Bash
Pillow: A modern fork of the Python Imaging Library (PIL). It is used for handling the distribution curve images and the camera preview images in the GUI.
Generated bash
python3 -m pip install Pillow
Use code with caution.
Bash
astral: Used by the Holy Grail controller to calculate the sun's position based on time and location. Note: The code is specifically written for the older but stable version 1.x of this library.
Generated bash
# Install the specific version the code is compatible with
python3 -m pip install astral==1.10.1
Use code with caution.
Bash
pytz: Required by astral for handling timezones correctly.
Generated bash
python3 -m pip install pytz
Use code with caution.
Bash
Note on "Externally Managed Environment"
If you are running a very new version of Raspberry Pi OS, pip might give you an "externally-managed-environment" error. In this case, your OS prefers you to install libraries using apt. You can try installing them with sudo apt-get install python3-numpy python3-pil python3-serial, etc., but creating a virtual environment is the best long-term solution to avoid these issues.
4. Included Python Libraries
The following modules are used by the project but are part of Python's standard library, so no installation is needed for them:
tkinter
threading
time
os
math
logging
enum
subprocess
json
