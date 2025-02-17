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
    cameraMatrix,dist,_,_ = pickle.load(f)


#initialize LCD
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
            newAngle = LCDqueue.get_nowait()#get the latest angle without waiting
            while not LCDqueue.empty():
                LCDqueue.get_nowait()#clear the queu of intermediate angles
            try:
                lcd.clear()
                lcd.message = f"Angle:\n{newAngle:.2f}"
            except Exception as e:
                print(f"Failed to update LCD: {e}")
        if endQueue:
            break


def find_center(corners):
    """
    Calculate the center coordinates of an ArUco marker.

    Args:
        corners (list): A list of corner points of the detected ArUco marker.

    Returns:
        tuple: The (x, y) coordinates of the marker's center.
    """
    for outline in corners:
        marker_corners = outline.reshape((4,2))
        
        # Compute the center of the marker
        center_x = int(np.mean(marker_corners[:, 0]))
        center_y = int(np.mean(marker_corners[:, 1]))

    return center_x, center_y

def find_phi(corners, cameraMatrix, distCoeffs):
    """
    Calculate the angle (phi) from the camera to the center of the ArUco marker.

    Args:
        corners (numpy.ndarray): The corner points of the detected ArUco marker.
        cameraMatrix (numpy.ndarray): The intrinsic camera matrix.
        distCoeffs (numpy.ndarray): The distortion coefficients.
        
    Returns:
        float: The calculated angle (phi) in degrees, relative to the center of the camera.
    """
    # Define 3D coordinates of the marker's corners (assuming the marker has a known size)
    marker_size = 0.05  # size of the marker in meters (adjust based on your marker size)
    marker_3d_points = np.array([
        [-marker_size / 2, -marker_size / 2, 0],  # Bottom-left corner
        [marker_size / 2, -marker_size / 2, 0],   # Bottom-right corner
        [marker_size / 2, marker_size / 2, 0],    # Top-right corner
        [-marker_size / 2, marker_size / 2, 0]    # Top-left corner
    ], dtype=np.float32)

    # Use solvePnP to get the rotation (rvec) and translation (tvec) vectors
    _, rvec, tvec = cv2.solvePnP(marker_3d_points, corners, cameraMatrix, distCoeffs)

    # Project the 3D points to 2D points on the image plane
    projected_points, _ = cv2.projectPoints(marker_3d_points, rvec, tvec, cameraMatrix, distCoeffs)
    
    # Find the center of the marker in 2D image space
    center_2d = np.mean(projected_points, axis=0).flatten()

    # Get the camera's optical center (principal point)
    cx, cy = cameraMatrix[0, 2], cameraMatrix[1, 2]

    # Calculate the angle between the camera center and the marker center in the image plane
    dx = center_2d[0] - cx  # Difference in x-coordinate
    dy = center_2d[1] - cy  # Difference in y-coordinate

    # Calculate the angle in the x-y plane (in radians) and convert to degrees
    phi = np.degrees(np.arctan2(dy, dx))
    
    return phi


def detect_marker_and_angle():
    """
    Detects ArUco markers in the camera feed, calculates their angle relative to the camera's center, and updates the LCD display.

    This function initializes the camera feed and starts an infinite loop to continuously capture frames. 
    It undistorts each frame, converts it to grayscale, and detects ArUco markers. 
    For each detected marker, it calculates the angle relative to the camera's center and updates the LCD display if the angle has changed. 
    The loop breaks if the 'q' key is pressed.

    Exceptions:
        Prints error messages if camera initialization or frame capture fails.
    """

    # initialize angle
    oldAngle = 0.00
    #start LCD thread
    LCDthread = threading.Thread(target = LCDdisplay, args=())
    LCDthread.start()

    # Open the camera feed
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return


    # Set the field of view (fov) of the camera (in degrees)

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

        # Convert the image to grayscale then apply adaptive threshold that helps exemplify contours for aruco detection
        gray = cv2.adaptiveThreshold(cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY), 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, myDict)
        
        if len(corners) > 0:
            # Draw the detected markers
            frame_undistorted = aruco.drawDetectedMarkers(frame_undistorted, corners, borderColor=(0,255,0))

            center = find_center(corners)

            # Mark the center of the marker on the frame
            cv2.circle(frame_undistorted, center, 3, (0, 255, 0), -1)

            # Calculate the angle of the marker relative to the camera's center
            newAngle = calc_angle_using_pose(rvec)
            ANGLE_THRESHOLD = 0.01
            if(abs(newAngle-oldAngle) >= ANGLE_THRESHOLD):
                oldAngle = newAngle

                if not LCDqueue.empty():
                    LCDqueue.get_nowait() #remove old value immediately
                LCDqueue.put(newAngle)


            # Display the angle text on the frame
            cv2.putText(frame_undistorted, f"{newAngle:.2f} degrees", (center[0] + 10, center[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

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