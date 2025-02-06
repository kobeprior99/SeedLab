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


#set up the lcd
#lcd setup
lcd_columns = 16
lcd_rows = 2 
#i2c setup
# i2c_lcd = board.I2C() 
# lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows) 
#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
# i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

#function to send data to the arduino
# def send_string(data1, data2, offset):
#     '''
#     Function to send a string to the arduino
#     '''
#     #convert the string to a list of ascii values list comprehension
#     command = [data1, data2]
#     #handle exception if i2c write fails
#     try:
#         #parameters are address of arduino, register to write to, and data to write
#         i2c_arduino.write_i2c_block_data(ARD_ADDR, offset, command)
#     except IOError:
#         print("Could not write data to the Arduino.")
#     #wait for a bit
#     sleep(.1)


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

    cv.imshow("quadrant_detect", colorFrame)

    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        cv.destroyAllWindows()
        break

#close all processes
camera.release()