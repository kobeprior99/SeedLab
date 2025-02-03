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
i2c = board.I2C() 
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows) 


#camera setup
camera = cv.VideoCapture(0)
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
