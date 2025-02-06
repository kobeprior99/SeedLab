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
Hardware Setup:  <TODO>
Example Excecution: <TODO>
'''
#import necessary libraries
import cv2 as cv
from smbus2 import SMBus
import numpy as np
from cv2 import aruco
from time import sleep
import board
import time
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

def LCDdisplay():
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows) 
    lcd.clear()
    while True:
        if not LCDqueue.empty():
            newLocation = LCDqueue.get()
            lcd.clear()
            lcd.message = "Desied Location:\n"+str(newLocation)

LCDthread = threading.Thread(target = LCDdisplay, args=())
LCDthread.start()
#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

# function to send coordinates to the arduino
def send_coordinates(coordinates):
    '''
    Function to send a string to the arduino
    '''

    #handle exception if i2c write fails
    try:
        #parameters are address of arduino, register to write to, and data to write
        i2c_arduino.write_i2c_block_data(ARD_ADDR, 0, coordinates)
    except IOError:
        print("Could not write data to the Arduino.")
def track_marker_quadrant(corners, width, height):
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
        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2))
            colorFrame = cv.putText(colorFrame, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2) 
        
    cv.imshow("quadrant_detect", colorFrame)
    # if (oldLocation != newLocation):
    #     send_coordinates(newLocation)
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        cv.destroyAllWindows()
        break

#close all processes
camera.release()
LCDthread.join()