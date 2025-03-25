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
* 03/06/2025    Kobe Prior Blane added the LCD display function
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
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
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
LOWER_GREEN = np.array([35, 100, 120])
UPPER_GREEN = np.array([85, 255, 255])
#note that the hue of red wraps around so we need two bounds
LOWER_RED1 = np.array([0, 150, 100 ])
UPPER_RED1 = np.array([5, 255 ,255 ])
LOWER_RED2 = np.array([175, 150, 100 ])
UPPER_RED2 = np.array([180, 255, 255 ])

#constant marker width in inches
MARKER_WIDTH_IRL = 2 #inches
#aruco dictrionary
MY_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # setup aruco dict


#LCD setup
lcd_columns = 16
lcd_rows = 2 
i2c_lcd = board.I2C()
data_lock = threading.Lock()
latest_data = {"angle": 0.0, "dist": 0.0, "arrow": -1}
endThread = False #flag to end inf loop in LCD display
try:
    # initialize LCD
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
    lcd.color = [100, 0, 0] #red
    lcd.clear()
except Exception as e:
    print(f"LCD initialization failed: {e}")
#set up special characters for LCD
#theta symbol
theta_char = [
    0b11111,  
    0b00100,   
    0b01110,  
    0b10101,  
    0b10101,  
    0b01110,  
    0b00100,    
    0b11111   
]
try:
    lcd.create_char(0, theta_char)  # Store the theta symbol at position 0
except Exception as e:
    print(f"Failed to create theta symbol: {e}")
#arrow symbol
arrow_char = [
    0b00000,
    0b00100,
    0b00010,
    0b11111,
    0b00010,
    0b00100,
    0b00000,
    0b00000
]
try:
    lcd.create_char(1, arrow_char)  # Store the theta symbol at position 1
except Exception as e:
    print(f"Failed to create arrow symbol: {e}")

def lcd_thread():
    """
    Thread function to update the LCD display with the latest data.

    This function runs in a separate thread and continuously updates the LCD display
    with the latest angle, distance, and arrow direction data. The thread terminates
    when the endThread flag is set to True.
    """
    while True:
        if endThread:
            try:
                lcd.clear()
            except Exception as e:
                print(f"Failed to clear LCD: {e}")
            break
        with data_lock: #lock while reading shared data
            data_copy = latest_data.copy() #copy the data
        try:
            lcd.clear()
            #example print ø: 18.00 →:L
            #               D:12.00
            angle_display = f"{data_copy['angle']:.2f}" if data_copy['angle'] is not None else "N/A"
            distance_display = f"{data_copy['dist']:.2f}" if data_copy['dist'] is not None else "N/A"
            lcd.message = f"\x00:{angle_display} \x01:{data_copy['arrow']}\ndist:{distance_display}"
            time.sleep(.5)
        except Exception as e:
            print(f"Failed to update LCD: {e}")
    return

#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

# global float array for data to send to arduino
instructions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#  [good_angle, angle, good_distance, distance, good_arrow, arrow]

def send_instructions():
    """
    Sends instructions to an Arduino via I2C communication.

    This function sends a list of float instructions to the Arduino. The instructions
    include information about the angle, distance, and arrow detection status.

    Raises:
    IOError: If the I2C write operation fails.
    """
    #handle exception if i2c write fails
    try:
        # instruction_array [good_angle, angle, good_distance, distance, good_arrow, arrow]
        # 1.0 is valid, 0.0 is invalid
        byte_array = bytearray()
        #floats have to be sent a special way and decoded in a special way
        for instruction in instructions:
            byte_array.extend(struct.pack("f", instruction))
        #debug
        #print(len(byte_array))
        #parameters are address of arduino, register to write to, and data to write
        i2c_arduino.write_i2c_block_data(ARD_ADDR, 0, list(byte_array))
    except IOError:
        print("Could not write data to the Arduino.")
        return 
    
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

def check_arrow(masks, frame, aruco_center):
    """
    Check for the presence of left or right arrows in the frame.

    Args:
        masks (tuple): A tuple containing the green mask and red mask.
        frame (ndarray): The image frame in which to detect arrows.
        aruco_center (tuple): The (x, y) coordinates of the ArUco marker's center.

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
            if x < aruco_center[0]:
                return 0
    #right red arrow
    for contour in red_contours:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, 'RIGHT', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
            if x > aruco_center[0]:
                return 1
    return -1

def find_center(corners, frame):
    """
    Calculate the center coordinates of an ArUco marker and draw a dot at the center.

    Args:
        corners (list): A list of corner points of the detected ArUco marker.
        frame (ndarray): The image frame where the marker is detected.

    Returns:
        tuple: The (x, y) coordinates of the marker's center.
    """
    for outline in corners:
        marker_corners = outline.reshape((4,2))
        
        # Compute the center of the marker
        center_x = int(np.mean(marker_corners[:, 0]))
        center_y = int(np.mean(marker_corners[:, 1]))
        #draw dot at the center of aruco marker
        cv2.circle(frame, (center_x,center_y), 3, (255, 255, 0), -1)

    return (center_x, center_y)

def distance(corners, ids, frame, center):
        """
        Calculate the distance from the camera to the detected ArUco marker and display it on the frame.
        Args:
            corners (list): List of detected marker corners.
            ids (numpy.ndarray): Array of detected marker IDs.
            frame (numpy.ndarray): The image frame where the markers are detected.
            center (tuple): The center coordinates (x, y) where the distance text will be displayed.
        Returns:
            float: The distance from the camera to the detected marker in inches.
        """
        
        _, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_WIDTH_IRL, cameraMatrix, dist)
        if ids is not None:
            for i in range(len(ids)):
                # Extract translation vector (tvec) to get distance
                distance_found = tvecs[i][0][2]  # Z-distance from camera to marker

                # Display distance on the image
                cv2.putText(frame, f"{distance_found:.2f} inches", (center[0], center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                # debug
                #print(f"Marker ID {ids[i][0]}: {distance_found:.2f} inches")
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
    global endThread
    #queue to store data to be sent to LCD
    # LCDthread = threading.Thread(target = lcd_thread, args=())
    # LCDthread.start()
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
            # if there is a marker detected, find the center, angle, distance, and arrow
            center = find_center(corners, frame_undistorted)
            instructions[0] = 1.0 #good_angle ->1.0
            instructions[1] = findPhi(center, frame_undistorted) #angle ->angle
            #debug:
            #print(angle)

            masks = find_mask(frame_undistorted)
            arrow = check_arrow(masks, frame_undistorted, center)
            if arrow == 0:
                #here we would modify instruction array
                #print("LEFT")
                instructions[4] = 1.0 #good_arrow ->1.0
                instructions[5] = 0.0 #arrow ->0.0
            elif arrow == 1:
                #print("RIGHT")
                instructions[4] = 1.0 #good_arrow ->1.0
                instructions[5] = 1.0 #arrow ->0.0
            else:
                #no arrow detected good_arrow ->0.0
                #print("NO ARROW DETECTED")
                instructions[4] = 0.0 #good_arrow ->0.0
            instructions[2] = 1.0 #good_distance ->1.0
            instructions[3] = distance(corners, ids, frame_undistorted, center) #distance ->distance
        else:
            #no aruco marker detected set all instructions to 0.0
            instructions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        #send instructions to arduino if they are not all 0.0
        if instructions != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]:
            send_instructions()
        #send only the most recent instructions to LCD
        with data_lock:
            if instructions[0] == 1.0:
                latest_data["angle"] = instructions[1]
            else:
                latest_data["angle"] = None
            if instructions[2] == 1.0:
                latest_data["dist"] = instructions[3]
            else:
                latest_data["dist"] = None
            if instructions[4] == 1.0 and instructions[5] == 0.0:
                latest_data["arrow"] = "L"
            if instructions[4] == 1.0 and instructions[5] == 1.0:
                latest_data["arrow"] = "R"
            if instructions[4] == 0.0:
                latest_data["arrow"] = "N"
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
    endThread = True
