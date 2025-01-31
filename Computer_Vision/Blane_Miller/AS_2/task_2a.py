'''
*******************************************************************
* File Name         : task2b.py
* Description       : take a picture of colors.pdf to detect green shapes, then perform
morphological transformations to clean up the mask.
*                    
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 01/27/2025	Blane Miller	Created File
*
******************************************************************
Hardware Setup: Power on the Pi -> either connect peripherials or connect via PiConnect or other VNC service --> connect web cam
Example Excecution: -> open terminal -> navigate to the directory where the task2b.py file is located using cd command ->
run the python file using 'python task1.py' command and point the camera towards the something that may contain green.
'''

#import libraries
import cv2
from cv2 import aruco
import numpy as np
from time import sleep
import board
import time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#lcd setup
lcd_columns = 16
lcd_rows = 2 

i2c = board.I2C() 

lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows) 

#camera setup
camera = cv2.VideoCapture(0)
sleep(.5)

#setup variables to let lcd only update upon seeing something new
last_marker_id = None
message_displayed = None

#inf loop for picture taking until waitkey press
while(True):
    ret, frame = camera.read() 

    myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # setup aruco dict
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale for ArUco detection

    #setup overlay display params
    corners, ids, rejected = aruco.detectMarkers(grey, myDict)
    overlay = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)
    overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor=4)

    #Look for arucos
    if ids is not None: 
        ids = ids.flatten()
        
        # Determine message based on number of detected markers
        if len(ids) == 1:
            message = f"The id is {ids[0]}"
        elif len(ids) > 1:
            delimiter = '&'
            message = f"ids are {delimiter.join(map(str, ids))}"
        else:
            message = "No markers found"

        # Update the LCD display only if the message has changed
        if message != message_displayed:
            lcd.clear()
            lcd.message = message
            message_displayed = message
            last_marker_id = ids[0]  # Store first ID for reference
        
        # Draw marker IDs on the image
        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2))
            overlay = cv2.putText(overlay, str(id), 
                                  (int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2) 
            
    # Make sure that the message starts as No markers found if no aruco is immediately detected
    else:
        if message_displayed != "No markers found":
            lcd.clear()
            lcd.message = "No markers found"
            message_displayed = "No markers found"
            last_marker_id = None

    #show overlay from earlier and get waitkey for inf loop break setup
    cv2.imshow("overlay", overlay)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        cv2.destroyAllWindows()
        break

camera.release()
