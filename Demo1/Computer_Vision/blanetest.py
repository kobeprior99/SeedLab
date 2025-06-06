'''
*******************************************************************
* File Name         : blanetest.py

* Description       : Fork of old Demo1.py code for experimentation
*
* Supplementary File(s): cam_cal.py used to generate intrinsic camera parameters stored in calibration.pkl file   

* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/16/2025    Blane Miller Made substantial progress: found phi calculation within .5 degrees
******************************************************************
Hardware Setup: 
- Raspberry Pi
- Pi Camera
- I2C LCD Display
- ArUco markers 5x5 cm per instructions on assignment

Example Execution: 
- Ensure the camera is calibrated and calibration.pkl is available.
- Run the script using: python blanetest.py after navigating to the correct directory
- Place an ArUco marker in front of the camera to see the angle displayed on the LCD and the live video feed.

PLEASE NOTE: this is draft code and some documentation is sparse because it will not be submitted for a deliverable, the goal of this file was to quickly develop.
'''

import cv2
from cv2 import aruco
import numpy as np
import pickle  # Using pickle to load the calibration data
import sys
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue

# Load the camera calibration results
with open('calibration.pkl', 'rb') as f:
    cameraMatrix,dist,rvec,tvec = pickle.load(f)

# with open('dist.pkl', 'rb') as f:
#     dist = pickle.load(f)
# Function to calculate the angle
def findPhi(fov, object_pixel, image_width, cx, fx):
    # Convert FOV to radians
    fov_rad = np.radians(fov)

    # Compute angle using arctan
    phi = np.degrees(np.arctan((object_pixel - cx) / fx))
    
    return round(phi,2)


lcd_columns = 16
lcd_rows = 2 

i2c_lcd = board.I2C()

LCDqueue = queue.Queue()
endQueue = False #flag to end inf loop in LCD display

def LCDdisplay():
    """
    Initializes and updates an LCD display with messages from a queue.

    This function attempts to initialize an LCD display using the 
    character_lcd.Character_LCD_RGB_I2C class. If initialization fails, 
    it prints an error message and exits the function. Once initialized, 
    the function enters an infinite loop where it checks for new messages 
    in the LCDqueue. If a new message is found, it updates the LCD display 
    with the message. The loop breaks if the endQueue flag is set.

    Exceptions:
        Prints error messages if LCD initialization or message update fails.
    """
    try:
        lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
        lcd.clear()
    except Exception as e:
        print(f"LCD initialization failed: {e}")
        return

    while True:
        if not LCDqueue.empty():
            newAngle = LCDqueue.get()
            try:
                lcd.clear()
                lcd.message = "Angle:\n" + str(newAngle)
            except Exception as e:
                print(f"Failed to update LCD: {e}")
        if endQueue:
            break

#start LCD thread
LCDthread = threading.Thread(target = LCDdisplay, args=())
LCDthread.start()

# Set up ArUco marker detection
def detect_marker_and_angle():
    oldAngle = 0
    # Open the camera feed
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Set the field of view (fov) of the camera (in degrees)
    fov = 68.5  # Example FOV; adjust this to your camera's specifications

    # ArUco dictionary and parameters
    myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    #parameters = aruco.DetectorParameters_create()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Undistort the frame using the camera matrix and distortion coefficients
        frame_undistorted = cv2.undistort(frame, cameraMatrix, dist)

        # Convert the image to grayscale
        gray = cv2.adaptiveThreshold(cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY), 255, 
                             cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(gray, myDict)
        
        if len(corners) > 0:
            # Draw the detected markers
            frame_undistorted = aruco.drawDetectedMarkers(frame_undistorted, corners, ids)

            for i, corner in enumerate(corners):
                # Get the coordinates of the ArUco marker's center
                marker_corners = corner.reshape(4, 2)
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))

                # Mark the center of the marker on the frame
                cv2.circle(frame_undistorted, (center_x, center_y), 5, (0, 255, 0), -1)

                # Calculate the angle of the marker relative to the camera's center
                height, width = frame.shape[:2]
                newAngle = findPhi(fov, center_x, width, cameraMatrix[0,2], cameraMatrix[0,0])
                if oldAngle != newAngle:
                    oldAngle = newAngle
                    LCDqueue.put(newAngle)


                # Display the angle text on the frame
                cv2.putText(frame_undistorted, f"{newAngle:.2f} degrees", (center_x + 10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Show the output frame with the detected marker and angle
        cv2.imshow('ArUco Marker Detection', frame_undistorted)

        # Break the loop on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_marker_and_angle()
    endQueue = True