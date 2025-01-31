import cv2
from cv2 import aruco
import numpy as np
from time import sleep
import board
import time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

lcd_columns = 16
lcd_rows = 2 

i2c = board.I2C() 

lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows) 

camera = cv2.VideoCapture(0)
sleep(.5)

last_marker_id = None
message_displayed = None

while(True):
    ret, frame = camera.read()

    myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale for ArUco detection

    corners, ids, rejected = aruco.detectMarkers(grey, myDict)
    overlay = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)
    overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor=4)

    if ids is not None: 
        ids = ids.flatten()
        
        # Determine message based on number of detected markers
        if len(ids) == 1:
            message = f"The id is {ids[0]}"
        elif len(ids) >= 2:
            message = f"ids are {ids[0]} & {ids[1]}"
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
    else:
        if message_displayed != "No markers found":
            lcd.clear()
            lcd.message = "No markers found"
            message_displayed = "No markers found"
            last_marker_id = None

    cv2.imshow("overlay", overlay)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        cv2.destroyAllWindows()
        break

camera.release()
