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

LOWER_GREEN = np.array([35, 100, 100])
UPPER_GREEN = np.array([85, 255, 255])

FX = cameraMatrix[0,0] # focal length in pixels
CX = cameraMatrix[0,2] # camera center in pixels

#aruco dictrionary
MY_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # setup aruco dict


#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

# constant width of aruco marker in inches
X = 2.0 #inches

# LCD Dimensions and I2C
lcd_columns = 16
lcd_rows = 2 

i2c_lcd = board.I2C()

LCDqueue = queue.Queue()
endQueue = False #flag to end inf loop in LCD display


def main():
    # put all functionality here
    return

#run the code
if __name__ == "__main__":
    main()