'''
*******************************************************************
* File Name         : arrow.py

* Description       : detect red-right arrow or green-left arrow and return 1 or 0 respectively
*
* Supplementary File(s): Computer_Vision/Demo1/cam_cal.py used to generate intrinsic camera parameters stored in calibration.pkl file  
* Kobe_Prior/AS2/assignment/task2.py to find contours

* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/22/2025    Kobe Prior and Blane Miller Created File
* 02/23/2025    Kobe Prior tested on laptop camera
******************************************************************
Hardware Setup: 
- Raspberry Pi
- Pi Camera
- I2C LCD Display
-print (ensure 2x2 inches): leftarrow.png, rightarrow.png

Example Execution: 
- Ensure calibration.pkl is available in working directory.
- Place an beacon marker in front of the camera
- Run the script using: python arrow.py after navigating to the correct directory
PLEASE NOTE: this is draft code and some documentation is sparse because it will not be submitted for a deliverable, the goal of this file was to quickly develop.
'''

import cv2
import numpy as np
from cv2 import aruco

#constant bounds for red and green
#note that the hue of red wraps around
LOWER_RED1 = np.array([0, 100, 100 ])
UPPER_RED1 = np.array([10, 255 ,255 ])
LOWER_RED2 = np.array([170, 100, 100 ])
UPPER_RED2 = np.array([180, 255, 255 ])

LOWER_GREEN = np.array([35, 100, 100])
UPPER_GREEN = np.array([85, 255, 255])


def find_mask(frame):
    #convert to frame to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
    red1_mask = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    red2_mask = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    red_mask = cv2.bitwise_or(red1_mask, red2_mask)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
    return (green_mask, red_mask)

def check_arrow(masks, frame, aruco_center):
    green_mask, red_mask = masks
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # left green arrow
    for contour in green_contours:
        if cv2.contourArea(contour) > 400:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, 'LEFT', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            if x < aruco_center[0]:
                return 0
    #right red arrow
    for contour in red_contours:
        if cv2.contourArea(contour) > 400:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, 'RIGHT', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            if x > aruco_center[0]:
                return 1
    return -1

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

def main():
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
        # Convert the image to grayscale then apply adaptive threshold that helps exemplify contours for aruco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kernel = np.array([[0, -1, 0],  
                        [-1, 5,-1],  
                        [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)
        corners, _, _ = aruco.detectMarkers(gray, myDict)
        if len(corners) > 0:
            frame = aruco.drawDetectedMarkers(frame, corners, borderColor=(0,255,0))
            center = find_center(corners)
            cv2.circle(frame, center, 3, (0, 255, 0), -1)
            masks = find_mask(frame)
            arrow = check_arrow(masks, frame, center)
            if arrow == 0:
                print("LEFT")
            elif arrow == 1:
                print("RIGHT")
        cv2.imshow('Arrow Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    #turn off the camera and destroy all windows
    cap.release()
    cv2.destroyAllWindows()
    return


#tested remotely using mac camera so didn't apply camera calibration
if __name__ == "__main__":
    main()