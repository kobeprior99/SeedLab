'''
*******************************************************************
* File Name         : miniProject.py
* Description       : 
*                    
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 02/03/2025	Kobe Prior and Blane Miller	Created File
*
******************************************************************
Hardware Setup:  Place LCD header on Pi, connect following headers to arduino:
-connect gnd to gnd, sda to sda, and scl to scl from pi to arduino.
-connect pin 3 on Pi to pin A4 on Arduino (SDA)
-connect pin 5 on Pi to pin A5 on Arduino (SCL)
-connect pin 6 on Pi to GND on Arduino
Connect camera to the pi
Example Excecution: navigate to the directory this file the type python MiniProject.py
'''

#import necessary libraries
import cv2 as cv
from smbus2 import SMBus
from cv2 import aruco
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue

myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # setup aruco dict

#set up the lcd
#lcd setup
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
            newLocation = LCDqueue.get()
            try:
                lcd.clear()
                lcd.message = "Desired Location:\n" + str(newLocation)
            except Exception as e:
                print(f"Failed to update LCD: {e}")
        if endQueue:
            break
#start LCD thread
LCDthread = threading.Thread(target = LCDdisplay, args=())
LCDthread.start()

#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

# function to send coordinates to the arduino
def send_coordinates(quadrant):
    '''
    Function to send a quadrant to string to the arduino
    '''
    #handle exception if i2c write fails
    try:
        coord_array = [quadrant[0], quadrant[1]]
        #parameters are address of arduino, register to write to, and data to write
        i2c_arduino.write_i2c_block_data(ARD_ADDR, 0, coord_array)
    except IOError:
        print("Could not write data to the Arduino.")


def track_marker_quadrant(corners, width, height):
    """
    Determines the quadrant of the image in which the marker is located.
    Args:
        corners (list): A list of arrays containing the corner points of the markers.
        width (int): The width of the image.
        height (int): The height of the image.
    Returns:
        tuple: A tuple representing the quadrant (quadrant_x, quadrant_y) where:
            - quadrant_x is 0 if the marker is in the left half, 1 if in the right half.
            - quadrant_y is 0 if the marker is in the top half, 1 if in the bottom half.
    """
    x_center = width // 2
    y_center = height // 2

    for outline in corners:
        markerCorners = outline.reshape((4,2))
        
        # Compute the center of the marker
        markerX = int((markerCorners[0,0] + markerCorners[2,0]) / 2)
        markerY = int((markerCorners[0,1] + markerCorners[2,1]) / 2)

        # Determine the quadrant
        quadrant_x = 1 if markerX > x_center else 0
        quadrant_y = 1 if markerY > y_center else 0
        quadrant = (quadrant_x, quadrant_y)
        # debug marker quadrant:
        # print(f"Marker Center: ({markerX}, {markerY}), Quadrant: {quadrant}")

    return quadrant

#camera setup
camera = cv.VideoCapture(0)
if not camera.isOpened():
    print("Error: Could not open video.")
    exit()
#modify height and width of the camera
camera.set(cv.CAP_PROP_FRAME_WIDTH, 424)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
sleep(.5)
oldLocation = (0,0)


while True:
    #while true loop to send information about the aruco markers location to the arduino
    ret, frame = camera.read() #read the camera frame
    #ensure camera is ready
    if not ret: 
        break

    #get heigth and width so we can determine center points
    height, width, _ = frame.shape
    x_center = width // 2
    y_center = height // 2

    # convert to grayscale
    grayScale = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # convert back to BGR to draw colored lines
    colorFrame = cv.cvtColor(grayScale, cv.COLOR_GRAY2BGR)
    # draw horizontal and vertical lines
    cv.line(colorFrame, (0, y_center), (width, y_center), (255, 0, 255), thickness=2)
    cv.line(colorFrame, (x_center, 0), (x_center, height), (255, 0, 255), thickness=2)

    corners, ids, rejected = aruco.detectMarkers(grayScale, myDict)
    colorFrame = aruco.drawDetectedMarkers(colorFrame, corners, borderColor=(0,255,0))
    if ids is not None: 
        ids = ids.flatten()
        newLocation = track_marker_quadrant(corners, width, height)
        
        if oldLocation != newLocation:
            oldLocation = newLocation
            send_coordinates(newLocation)
            LCDqueue.put(newLocation)

        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2))
            # place idea above aruco marker
            colorFrame = cv.putText(colorFrame, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2) 
        
    cv.imshow("quadrant_detect", colorFrame)
    #destroy all windows and break inf loop if q pressed.
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        cv.destroyAllWindows()
        break

#close all processes
camera.release()
endQueue = True