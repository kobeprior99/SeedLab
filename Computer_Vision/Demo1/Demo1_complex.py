"""
Author: Kobe Prior
Date: 02-16-2024
Description: This script detects ArUco markers in a camera feed,
 calculates their angle relative to the camera's center,
   and displays the angle on an LCD screen.

This should be more complex and thereby more accurate than Demo1.py
"""
#import necessary libraries
import cv2
from cv2 import aruco
import numpy as np
import pickle  # Using pickle to load the calibration data
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue

# Load the camera calibration results
with open('calibration.pkl', 'rb') as f:
    cameraMatrix, dist, _, _ = pickle.load(f)

# Initialize LCD
lcd_columns = 16
lcd_rows = 2

i2c_lcd = board.I2C()

LCDqueue = queue.Queue()
endQueue = False  # Flag to end inf loop in LCD display

def LCDdisplay():
    try:
        lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
        lcd.clear()
    except Exception as e:
        print(f"LCD initialization failed: {e}")
        return

    while True:
        if not LCDqueue.empty():
            newAngle = LCDqueue.get_nowait()  # Get the latest angle without waiting
            while not LCDqueue.empty():
                LCDqueue.get_nowait()  # Clear the queue of intermediate angles
            try:
                lcd.clear()
                lcd.message = f"Angle:\n{newAngle:.2f}"
            except Exception as e:
                print(f"Failed to update LCD: {e}")
        if endQueue:
            break

def find_center(corners):
    for outline in corners:
        marker_corners = outline.reshape((4, 2))
        center_x = int(np.mean(marker_corners[:, 0]))
        center_y = int(np.mean(marker_corners[:, 1]))
    return center_x, center_y

def find_phi(rvec, tvec):
    # Calculate the angle (phi) from the camera to the center of the ArUco marker
    R, _ = cv2.Rodrigues(rvec)
    camera_position = -np.matrix(R).T * np.matrix(tvec)
    phi = np.degrees(np.arctan2(camera_position[0][0], camera_position[2][0]))
    return phi

def detect_marker_and_angle():
    oldAngle = 0.00
    LCDthread = threading.Thread(target=LCDdisplay, args=())
    LCDthread.start()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break

        frame_undistorted = cv2.undistort(frame, cameraMatrix, dist)
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, myDict)

        if len(corners) > 0:
            frame_undistorted = aruco.drawDetectedMarkers(frame_undistorted, corners, borderColor=(0, 255, 0))
            center = find_center(corners)
            cv2.circle(frame_undistorted, center, 3, (0, 255, 0), -1)

            marker_size = 0.05
            marker_3d_points = np.array([
                [-marker_size / 2, -marker_size / 2, 0],
                [marker_size / 2, -marker_size / 2, 0],
                [marker_size / 2, marker_size / 2, 0],
                [-marker_size / 2, marker_size / 2, 0]
            ], dtype=np.float32)

            for outline in corners:
                marker_corners = outline.reshape((4, 2))
                _, rvec, tvec = cv2.solvePnP(marker_3d_points, marker_corners, cameraMatrix, dist)
                newAngle = find_phi(rvec, tvec)
                ANGLE_THRESHOLD = 0.01
                if abs(newAngle - oldAngle) >= ANGLE_THRESHOLD:
                    oldAngle = newAngle
                    if not LCDqueue.empty():
                        LCDqueue.get_nowait()
                    LCDqueue.put(newAngle)
                cv2.putText(frame_undistorted, f"{newAngle:.2f} degrees", (center[0] + 10, center[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow('ArUco Marker Detection', frame_undistorted)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_marker_and_angle()
    endQueue = True
