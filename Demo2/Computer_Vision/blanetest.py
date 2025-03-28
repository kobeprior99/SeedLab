'''
*******************************************************************
* File Name         : Demo2.py

* Description       : Interpret the angle of an Aruco marker, 
* the distance to the marker, and the color of the arrow next to the marker. 
* When this information is available send it to the LCD screen and arduino.
*
* Supplementary File(s): Computer_Vision/Demo1/cam_cal.py used to generate intrinsic camera parameters stored in calibration.pkl file 
* distance.py and chatdist.py used to experiment with distance calculation function
* arrow.py used to experiment with arrow detection function
* test_float_pack.py file used to experiment with sending floats to arduino
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/22/2025    Kobe Prior and Blane Miller Created File
* 02/23/2025    Kobe Prior added necessary libraries and some initializations
* 03/03/2025    Kobe Prior Blane added the main function
* 03/06/2025    Kobe Prior Blane added the LCD display function
* 03/24/2025    Kobe Prior Blane added Revised LCD program to send 11 bytes instead of 24 at a time
******************************************************************
Hardware Setup: 
- Raspberry Pi
- Pi Camera
- I2C LCD Display
-I2C Arduino
-connect pin 3 on Pi to pin A4 on Arduino (SDA)
-connect pin 5 on Pi to pin A5 on Arduino (SCL)
-connect pin 6 on Pi to GND on Arduino
-print (ensure 2x2 inches): leftarrow.png, rightarrow.png

Example Execution: 
- Ensure calibration.pkl is available in working directory.
- Run the script using: python Demo2.py after navigating to the correct directory
- print 2x2 inch aruco markers and leftarrow.png and rightarrow.png
- Place an left or right 'beacon' 
'''

#import necessary libraries
import cv2
import numpy as np
import struct
import smbus2
import threading
from queue import Queue 
import board
import cv2.aruco as aruco
import pickle
import time
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus2 import SMBus


# Constants
ARD_ADDR = 8
MY_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Camera Calibration Parameters (Replace with actual values)
cameraMatrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  
dist = np.array([0, 0, 0, 0, 0])  

# Global Variables
frame_queue = Queue(maxsize=1)  # Holds only the latest frame
instructions = {"good_angle": 0, "angle": 0.0, "good_distance": 0, "distance": 0.0, "arrow": -1}
last_instructions = None

# Initialize I2C
i2c_arduino = smbus2.SMBus(1)  

def capture_frames(cap):
    """Continuously capture frames and put them into a queue."""
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if not frame_queue.empty():
            frame_queue.get_nowait()  # Remove old frame
        frame_queue.put(frame)  # Store latest frame

def find_center(corners, frame):
    """Find the center of detected ArUco marker."""
    return np.mean(corners[0][0], axis=0).astype(int)

def findPhi(center, frame):
    """Calculate angle from marker center."""
    return (center[0] - frame.shape[1] / 2) * 0.1  

def distance(corners, ids, frame, center):
    """Estimate distance from marker."""
    return 50.0 / (corners[0][0][1][1] - corners[0][0][0][1])  

def find_mask(frame):
    """Find red and green arrow masks."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_lower = np.array([0, 120, 70])
    red_upper = np.array([10, 255, 255])
    green_lower = np.array([50, 100, 100])
    green_upper = np.array([70, 255, 255])
    
    return cv2.inRange(hsv, green_lower, green_upper), cv2.inRange(hsv, red_lower, red_upper)

def check_arrow(masks, frame):
    """Detect arrows and determine direction."""
    green_mask, red_mask = masks
    min_contour_area = 1000  

    for mask, color in [(green_mask, (0, 255, 0)), (red_mask, (0, 0, 255))]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            if cv2.contourArea(contour) < min_contour_area:
                continue  

            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / float(h)

            if 1.5 < aspect_ratio < 3.5:
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                return 0 if color == (0, 255, 0) else 1  

    return -1  

def send_instructions():
    """Send instructions to Arduino via I2C."""
    global instructions, last_instructions
    if instructions == last_instructions:
        return  

    try:
        instruction_array = [
            instructions["good_angle"], instructions["good_distance"], instructions["arrow"]
        ]
        instruction_array += list(struct.pack('f', instructions["angle"]))
        instruction_array += list(struct.pack('f', (instructions["distance"] + 6.0)))

        i2c_arduino.write_i2c_block_data(ARD_ADDR, 0, instruction_array)
        last_instructions = instructions.copy()  
    except IOError:
        print("Write fail.\n")

def main():
    global instructions
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Precompute undistortion map
    map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix, dist, None, cameraMatrix, (640, 480), 5)

    # Start capture thread
    capture_thread = threading.Thread(target=capture_frames, args=(cap,), daemon=True)
    capture_thread.start()

    while True:
        if frame_queue.empty():
            continue  

        frame = frame_queue.get()
        frame_undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

        # ArUco marker detection
        gray = frame_undistorted  # Already grayscale due to CAP_PROP_CONVERT_RGB\
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, MY_DICT)

        if len(corners) > 0:
            center = find_center(corners, frame_undistorted)
            instructions["good_angle"] = 1  
            instructions["angle"] = findPhi(center, frame_undistorted)  
            instructions["good_distance"] = 1  
            instructions["distance"] = distance(corners, ids, frame_undistorted, center)  
        else:
            instructions["good_angle"] = 0
            instructions["good_distance"] = 0

        # Arrow detection
        masks = find_mask(frame_undistorted)
        instructions["arrow"] = check_arrow(masks, frame_undistorted)
        instructions["angle"] = float(instructions.get("angle", 0.0))
        instructions["distance"] = float(instructions.get("distance", 0.0))
        send_instructions()
        cv2.imshow('Demo2', frame_undistorted)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

