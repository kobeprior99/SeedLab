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

while(True):
    ret, frame = camera.read()

    myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #make the image greyscale for aruco detection
    # cv2.imshow("overlay", grey)

    corners, ids, rejected = aruco.detectMarkers(grey, myDict)
    overlay = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)
    overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor = 4)

    if not ids is None: 
        ids = ids.flatten()
        current_marker_id = ids[0] 
        if current_marker_id != last_marker_id:
            lcd.clear()
            lcd.message = f"The id is {current_marker_id}"
            last_marker_id = current_marker_id
        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2))
            overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2) 
    else:
            lcd.clear()
            lcd.message = "No markers found"
            last_marker_id = None
    cv2.imshow("overlay",overlay)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        cv2.destroyAllWindows()
        break
camera.release()