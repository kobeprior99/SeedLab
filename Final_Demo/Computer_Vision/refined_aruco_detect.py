'''
*******************************************************************
* File Name         : refined_aruco_detect.py

* Description       : A fork of Final_Demo/FinalDemo.py
*
* Supplementary File(s): 
* - Computer_Vision/Demo1/cam_cal.py: Generates intrinsic camera parameters stored in calibration.pkl.
* - Computer_Vision/Demo2/Demo2.py: the program modified for final Demo objevies.
*
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 4/7/2025 Kobe created a fork of Final_Demo/FinalDemo.py to use make Aruco detection more robust without doing adaptive thresholding.
* 4/7/2025 Kobe computed centers as an array of tuples to handle multiple markers. Then found closest marker and reported it
******************************************************************
Hardware Setup: 
- Raspberry Pi
- Pi Camera
- I2C Arduino
- Connections:
    - Connect pin 3 on Pi to pin A4 on Arduino (SDA).
    - Connect pin 5 on Pi to pin A5 on Arduino (SCL).
    - Connect pin 6 on Pi to GND on Arduino.
- Print (ensure 2x2 inches): leftarrow.png, rightarrow.png.

Example Execution: 
- Ensure calibration.pkl is available in the working directory.
- Run the script using: python Demo2.py after navigating to the correct directory.
- Print 2x2 inch ArUco markers and leftarrow.png/rightarrow.png.
- Place a left or right 'beacon' 5 to 6 feet away from the robot, aligned with its axis of rotation, and power on the robot.
'''


#import necessary libraries
import cv2
import numpy as np
import cv2.aruco as aruco
import pickle
import time
from time import sleep
import threading
from smbus2 import SMBus
import struct


#get camera intrisnic parameters
with open('calibration.pkl', 'rb') as f:
    cameraMatrix,dist,_,_ = pickle.load(f)

# gather useful constants from camera matrix
FX = cameraMatrix[0,0] # focal length in pixels
CX = cameraMatrix[0,2] # camera center in pixels


#constant bounds for red and green
LOWER_GREEN = np.array([40, 100, 30])
UPPER_GREEN = np.array([70, 150, 80])

#note that the hue of red wraps around so we need two bounds
LOWER_RED1 = np.array([0, 150, 50 ])
UPPER_RED1 = np.array([5, 255 ,140 ])
LOWER_RED2 = np.array([175, 150, 50 ])
UPPER_RED2 = np.array([180, 255, 140 ])

#constant marker width in inches
MARKER_WIDTH_IRL = 2 #inches
#aruco dictrionary
MY_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # setup aruco dict


#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

program_running = True # global variable to terminate threads when set to false

# global float array for data to send to arduino
instructions = {"marker_found": 0, "arrow": 2, "angle": 0.0, "distance": 0.0}
# arrow = -1 means no arrow detected, 0 means left arrow, 1 means right arrow
# marker_found = 0 means no marker detected, 1 means marker detected

instructions_lock = threading.Lock() #prevent race condiitons when accessing instructions
def send_instructions():
    """
    Sends instructions to an Arduino via I2C communication.

    This function sends 10 bytes: good marker, arrow, angle, distance
    Angle and distance are floats so are packaged as 4 bytes each.

    Raises:
    IOError: If the I2C write operation fails.
    """
    #handle exception if i2c write fails
    global instructions
    global program_running
    fail_count = 0 # initialize fail count for debugging
    while program_running == True:
        with instructions_lock:
            try:
                #just 2 bytes, note if good_marker is true then good_distance is also true
                instruction_array = [
                    instructions["marker_found"],
                    instructions["arrow"]
                ] # this will be 3 bytes, 1 byte each for good_angle, good_distance, and arrow
                
                #special handling for floats which will require an extra 8 bytes 4 bytes for each float
                angle_in_bytes = list(struct.pack('f', instructions["angle"]))
                distance_in_bytes = list(struct.pack('f',(instructions["distance"] + 6.0))) #add 6 to distance to
                instruction_array += angle_in_bytes + distance_in_bytes
                #debug: print(instruction_array)
                i2c_arduino.write_i2c_block_data(ARD_ADDR, 0, instruction_array)
                #    print("Instructions sent to Arduino.")
                time.sleep(0.02)#sending about 30 times a second should be fast enough for arduino to process
            except IOError:
                print(f"Write fail.{fail_count}\n")
                fail_count += 1
    
def find_mask(frame):
    """
    Generate masks for detecting red and green colors in the given frame.
    Helper function for check_arrow.
    Args:
        frame (ndarray): The image frame in which to detect colors.

    Returns:
        tuple: A tuple containing the green mask and red mask.
    """
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

def check_arrow(masks, frame):
    """
    Check for the presence of left or right arrows in the frame.

    Args:
        masks (tuple): A tuple containing the green mask and red mask.
        frame (ndarray): The image frame in which to detect arrows.

    Returns:
        int: 0 if a left arrow is detected, 1 if a right arrow is detected, -1 if no arrow is detected.
    """
    green_mask, red_mask = masks
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # left green arrow
    for contour in green_contours:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, 'LEFT', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
            return 0
        
    #right red arrow
    for contour in red_contours:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, 'RIGHT', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
            return 1
    return -1
#modified from previous code it is now find_centers since it scales to multiple markers when necessary
def find_centers(corners, frame):
    """
    Calculate the center coordinates of all ArUco markers on screen and draw a dot at the center.

    Args:
        corners (list): A list of corner points of the detected ArUco markers.
        frame (ndarray): The image frame where the markers are detected.

    Returns:
        list: A list of tuples, where each tuple contains the (x, y) coordinates of a marker's center.
    """
    centers = [] #array to hold the centers of all markers
    for outline in corners:
        marker_corners = outline.reshape((4,2))
        
        # Compute the center of the marker
        center_x = int(np.mean(marker_corners[:, 0]))
        center_y = int(np.mean(marker_corners[:, 1]))
        #draw dot at the center of aruco marker
        cv2.circle(frame, (center_x,center_y), 3, (255, 255, 0), -1)
        centers.append((center_x, center_y)) #append the center to the array

    return centers

def distance(corner, frame, center):
    """
    Calculate the distance of an ArUco marker from the camera.

    Args:
        corner (list): The corner points of the detected ArUco marker.
        frame (ndarray): The image frame where the marker is detected.
        center (tuple): The (x, y) coordinates of the marker's center.

    Returns:
        float: The calculated distance of the marker from the camera in inches.
    """
    
    _, tvecs, _ = aruco.estimatePoseSingleMarkers([corner], MARKER_WIDTH_IRL, cameraMatrix, dist)
    # Extract translation vector (tvec) to get distance
    distance_found = tvecs[0][0][2]  # Z-distance from camera to marker
    # Display distance on the image
    cv2.putText(frame, f"{distance_found:.2f} inches", (center[0], center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
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
    object_pixel = center[0] # note that this is the x coordinate of the center of the marker
    phi = np.degrees(np.arctan((CX - object_pixel) / FX))
    cv2.putText(frame, f'Angle {phi:.2f}', (center[0], center[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    return phi

def main():
    """
    Main function to capture video, detect ArUco markers, calculate their angles and distances, and send instructions to an Arduino.

    This function initializes the camera, captures video frames, detects ArUco markers, calculates their angles and distances,
    checks for the presence of arrows, and sends the relevant instructions to an Arduino via I2C communication. The processed
    video frames are displayed in a window.

    Raises:
    IOError: If the camera cannot be opened or a frame cannot be captured.
    """
    global instructions
    send_thread = threading.Thread(target = send_instructions)
    send_thread.start()  # Start the thread to send instructions to Arduino

    # Set up the camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25); #manual camera expsure
    cap.set(cv2.CAP_PROP_EXPOSURE, 2)  # Adjust this value (negative for some cameras)
    cap.set(cv2.CAP_PROP_FPS, 30) # set frames per second for the camera

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    #camera warm up
    time.sleep(2.0)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)) #contrast limited adaptive histogram equalization
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break
        #undistort the frame        
        frame_undistorted = cv2.undistort(frame, cameraMatrix, dist)
        #convert frame to grayscale
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        #apply histogram equalization to the grayscale frame
        gray = cv2.equalizeHist(gray)
        #apply CLAHE to the grayscale frame
        gray = clahe.apply(gray)

        corners, _, _ = aruco.detectMarkers(gray, MY_DICT)
        if len(corners) > 0:
            # if there are markers detected, find the centers, angles, distances, and arrows
            #report only the closest marker


            #create empty arrays to hold the distances and angles
            distances = []
            angles = []

            centers = find_centers(corners, frame_undistorted)

            #loop through all the markers and calculate the distance and angle
            for i, center in enumerate(centers):
                #calculate the distances and angles for each marker
                distances.append(distance(corners[i], frame_undistorted, center))
                angles.append(findPhi(center, frame_undistorted))

            #get the closest marker
            min_distance_index = distances.index(min(distances))
            instructions["marker_found"] = 1 #acceptable marker on screen
            instructions["angle"] = angles[min_distance_index] #report the angle of the closest marker
            #debug:
            #print(instructions["angle"])

            instructions["distance"] = distances[min_distance_index] #report the small distance
            #debug:
            #print(instructions["distance"])

        else:
            #if there is no marker, set the angle and distance to 0
            instructions["marker_found"] = 0 # no marker on screen

        # check if there is an arrow and change instructions, if there is an arrow and no marker we still want to turn 
        masks = find_mask(frame_undistorted)
        arrow = check_arrow(masks, frame_undistorted)
        if arrow == 0:
            #here we would modify instruction array
            #print("LEFT")
            instructions["arrow"] = 0 #->arrow is now left
        elif arrow == 1:
            #print("RIGHT")
            instructions["arrow"] = 1 #-> arrow is now right
        else:
            #no arrow detected good_arrow ->0.0
            #print("NO ARROW DETECTED")
            instructions["arrow"] = 2 #good_arrow ->0.0

        #display frame with all overlays
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
    program_running = False # set to false to terminate the threads, same thing as a daemon thread
