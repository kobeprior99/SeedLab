'''
*******************************************************************
* File Name         : Demo2.py

* Description       : Interpret the angle of an Aruco marker, 
* the distance to the marker, and the color of the arrow next to the marker. 
* When this information is available send it to the LCD screen and arduino.
*
* Supplementary File(s): Computer_Vision/Demo1/cam_cal.py used to generate intrinsic camera parameters stored in calibration.pkl file 
* distance.py used to experiment with distance calculation function
* arrow.py used to experiment with arrow detection function  
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/22/2025    Kobe Prior and Blane Miller Created File
* 02/23/2025    Kobe Prior added necessary libraries and some initializations
* 03/03/2025    Kobe Prior Blane added the main function
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
import board
import cv2.aruco as aruco
import pickle
import time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
from smbus2 import SMBus



#get camera intrisnic parameters
with open('calibration.pkl', 'rb') as f:
    cameraMatrix,dist,_,_ = pickle.load(f)

#define constants
#constant bounds for red and green
#note that the hue of red wraps around
LOWER_RED1 = np.array([0, 150, 100 ])
UPPER_RED1 = np.array([5, 255 ,255 ])
LOWER_RED2 = np.array([175, 150, 100 ])
UPPER_RED2 = np.array([180, 255, 255 ])

LOWER_GREEN = np.array([35, 120, 120])
UPPER_GREEN = np.array([85, 255, 255])

FX = cameraMatrix[0,0] # focal length in pixels
CX = cameraMatrix[0,2] # camera center in pixels

MARKER_WIDTH_IRL = 2 #inches
#aruco dictrionary
MY_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # setup aruco dict


#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

# constant width of aruco marker in inches

# LCD Dimensions and I2C
lcd_columns = 16
lcd_rows = 2 

i2c_lcd = board.I2C()

LCDqueue = queue.Queue()
endQueue = False #flag to end inf loop in LCD display


def find_mask(frame):
    #convert to frame to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    #GaussianBlur takes less resources than two morphological operations
    green_mask = cv2.GaussianBlur(green_mask, (5,5), 0)
    red1_mask = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    red2_mask = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    #add the upper and lower masks
    red_mask = cv2.bitwise_or(red1_mask, red2_mask)
    red_mask = cv2.GaussianBlur(red_mask, (5,5), 0)
    #debug to refine masks
    #cv2.imshow('red', red_mask)
    # cv2.imshow('green', green_mask)
    return (green_mask, red_mask)

def check_arrow(masks, frame, aruco_center):
    green_mask, red_mask = masks
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # left green arrow
    for contour in green_contours:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, 'LEFT', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
            if x < aruco_center[0]:
                return 0
    #right red arrow
    for contour in red_contours:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, 'RIGHT', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
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

def distance(corners, ids, frame, center):
        _, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_WIDTH_IRL, cameraMatrix, dist)
        if ids is not None:
            for i in range(len(ids)):
                # Extract translation vector (tvec) to get distance
                distance_found = tvecs[i][0][2]  # Z-distance from camera to marker

                # Display distance on the image
                cv2.putText(frame, f"{distance_found:.2f} inches", (center[0], center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                # debug
                print(f"Marker ID {ids[i][0]}: {distance_found:.2f} inches")
                return distance_found
                
def findPhi(center, frame):
    """
    Calculate the angle (phi) of an object relative to the camera's center.

    Args:
        object_pixel (int): The x-coordinate of the object's pixel position in the image.
    Returns:
        float: The calculated angle (phi) in degrees, rounded to two decimal places.
    """
    # Compute angle using arctan
    #return positive angle when marker is left of camera axis, negative when right
    object_pixel = center[0]
    phi = np.degrees(np.arctan((CX - object_pixel) / FX))
    cv2.putText(frame, f'Angle {phi:.2f}', (center[0], center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    return phi

def main():
    # put all functionality here
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    #camera warm up
    time.sleep(2)
    for _ in range(5):
        cap.read()

    myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break
        # Convert the image to grayscale then apply adaptive threshold that helps exemplify contours for aruco detection
        
        frame_undistorted = cv2.undistort(frame, cameraMatrix, dist)
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, myDict)
        if len(corners) > 0:
            center = find_center(corners)
            angle = findPhi(center, frame_undistorted)

            #debug:
            print(angle)

            cv2.circle(frame_undistorted, center, 3, (255, 255, 0), -1)
            masks = find_mask(frame_undistorted)
            arrow = check_arrow(masks, frame_undistorted, center)
            if arrow == 0:
                #here we would modify instruction array
                print("LEFT")
            elif arrow == 1:
                print("RIGHT")
            else:
                #no arrow detected good_arrow ->0.0
                print("NO ARROW DETECTED")
            distance(corners, ids, frame_undistorted, center)
        cv2.imshow('Demo2', frame_undistorted)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    #turn off the camera and destroy all windows
    cap.release()
    cv2.destroyAllWindows()
    return

#run the code
if __name__ == "__main__":
    main()