'''
*******************************************************************
* File Name         : distance.py

* Description       : Calculate the distance to an aruco marker using the size of the aruco marker in the image
*
* Supplementary File(s): cam_cal.py used to generate intrinsic camera parameters stored in calibration.pkl file   

* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/22/2025    Kobe Prior and Blane Miller Created File
* 02/24/2025    Blane Miller began adding distance calc fxn
******************************************************************
Hardware Setup: 
- Raspberry Pi
- Pi Camera
- I2C LCD Display
- ArUco markers 2x2 inch per instructions on assignment

Example Execution: 
- Ensure calibration.pkl is available in working directory.
- Run the script using: python distance.py after navigating to the correct directory
- Place an ArUco marker in front of the camera
PLEASE NOTE: this is draft code and some documentation is sparse because it will not be submitted for a deliverable, the goal of this file was to quickly develop.
'''

import cv2
from cv2 import aruco
import numpy as np
from time import sleep
import pickle  # Using pickle to load the calibration data

# Load the camera calibration results
with open('calibration.pkl', 'rb') as f:
    cameraMatrix,dist,_,_ = pickle.load(f)

fx = cameraMatrix[0,0]

def find_marker_width(corners):
    """
    Calculate the width of an ArUco marker by averaging the top and bottom edge widths.

    Args:
        corners (list): A list of corner points of the detected ArUco marker.

    Returns:
        float: The estimated width of the marker.
    """
    for outline in corners:
        marker_corners = outline.reshape((4,2))
        
        # Extract the top-left, top-right, bottom-right, and bottom-left corners
        top_left, top_right, bottom_right, bottom_left = marker_corners
        
        # Compute the width of the top and bottom edges
        top_width = np.linalg.norm(top_right - top_left)
        bottom_width = np.linalg.norm(bottom_right - bottom_left)
        
        # Compute the mean width
        mean_width = (top_width + bottom_width) / 2
        
    return mean_width

myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    #parameters = aruco.DetectorParameters_create()
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

    return (center_x, center_y)

def distance(width):
    distance = (fx*2)/width
    return distance

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")

    #start infinite loop press q to quit
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Undistort the frame using the camera matrix and distortion coefficients
    frame_undistorted = cv2.undistort(frame, cameraMatrix, dist)

    # Convert the image to grayscale then apply adaptive threshold that helps exemplify contours for aruco detection
    # gray = cv2.adaptiveThreshold(cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY), 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
    #sharpening filter
    kernel = np.array([[0, -1, 0],  
                    [-1, 5,-1],  
                    [0, -1, 0]])
    gray = cv2.filter2D(gray, -1, kernel)
    corners, ids, _ = aruco.detectMarkers(gray, myDict)
    if len(corners) > 0:
        width_actual = find_marker_width(corners)
        distance_found = distance(width_actual)
        center = find_center(corners)
        cv2.putText(frame_undistorted, f"{distance_found:.2f} inches", (center[0] + 10, center[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        print(distance_found)

    cv2.imshow("distance", frame_undistorted)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
