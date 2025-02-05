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
i2c_lcd = board.I2C() 
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows) 
#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

#function to send data to the arduino
def send_string(data1, data2 offset):
    '''
    Function to send a string to the arduino
    '''
    #convert the string to a list of ascii values list comprehension
    command = [data1, data2]
    #handle exception if i2c write fails
    try:
        #parameters are address of arduino, register to write to, and data to write
        i2c_arduino.write_i2c_block_data(ARD_ADDR, offset, command)
    except IOError:
        print("Could not write data to the Arduino.")
    #wait for a bit
    sleep(.1)


#camera setup
camera = cv.VideoCapture(0)
#modify height and width of the camera
sleep(.5)


while True:
    #while true loop to send information about the aruco markers location to the arduino
    ret, frame = camera.read() #read the camera frame
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        cv.destroyAllWindows()
        break

#close all processes
camera.release()