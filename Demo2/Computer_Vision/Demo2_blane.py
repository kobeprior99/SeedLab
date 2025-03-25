'''
*******************************************************************
* File Name         : Demo2.py
* Description       : Interpret the angle of an Aruco marker, 
* the distance to the marker, and the color of the arrow next to the marker.
* When this information is available, send it to the LCD screen and Arduino.
*******************************************************************
'''
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

# Load camera intrinsic parameters
with open('calibration.pkl', 'rb') as f:
    cameraMatrix, dist, _, _ = pickle.load(f)

FX = cameraMatrix[0, 0]  
CX = cameraMatrix[0, 2]  

LOWER_GREEN = np.array([35, 100, 120])
UPPER_GREEN = np.array([85, 255, 255])
LOWER_RED1 = np.array([0, 150, 100])
UPPER_RED1 = np.array([5, 255, 255])
LOWER_RED2 = np.array([175, 150, 100])
UPPER_RED2 = np.array([180, 255, 255])

MARKER_WIDTH_IRL = 2  
MY_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# LCD Setup
lcd_columns = 16
lcd_rows = 2 
i2c_lcd = board.I2C()
data_lock = threading.Lock()
latest_data = {"angle": None, "dist": None, "arrow": "N"}
endThread = False  

try:
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
    lcd.color = [100, 0, 0]  
    lcd.clear()
except Exception as e:
    print(f"LCD initialization failed: {e}")
    lcd = None  

def lcd_thread():
    while True:
        if endThread:
            if lcd:
                lcd.clear()
            break
        with data_lock:
            data_copy = latest_data.copy()
        try:
            if lcd:
                lcd.clear()
                angle_display = f"{data_copy['angle']:.2f}" if data_copy['angle'] is not None else "N/A"
                distance_display = f"{data_copy['dist']:.2f}" if data_copy['dist'] is not None else "N/A"
                lcd.message = f"\x00:{angle_display} \x01:{data_copy['arrow']}\ndist:{distance_display}"
            time.sleep(1)
        except Exception as e:
            print(f"Failed to update LCD: {e}")

ARD_ADDR = 8  
i2c_arduino = SMBus(1)

instructions = [0.0] * 6
last_instructions = instructions.copy() 

def send_instructions():
    try:
        byte_array = bytearray()
        for instruction in instructions:
            byte_array.extend(struct.pack("f", instruction))
        if instructions != last_instructions:
            i2c_arduino.write_i2c_block_data(ARD_ADDR, 0, list(byte_array))
    except IOError:
        print("Could not write data to the Arduino.")

def find_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    green_mask = cv2.GaussianBlur(green_mask, (5,5), 0)
    red1_mask = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    red2_mask = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    red_mask = cv2.bitwise_or(red1_mask, red2_mask)
    red_mask = cv2.GaussianBlur(red_mask, (5,5), 0)
    return green_mask, red_mask

def distance(corners, ids):
    if ids is not None and len(corners) > 0:
        _, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_WIDTH_IRL, cameraMatrix, dist)
        return tvecs[0][0][2]
    return None

def findPhi(center):
    object_pixel = center[0]
    phi = np.degrees(np.arctan((object_pixel - CX) / FX))
    return round(phi, 2)

def main():
    LCDthread = threading.Thread(target=lcd_thread)
    LCDthread.start()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    time.sleep(2)
    for _ in range(5):
        cap.read()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break
        
        frame_undistorted = cv2.undistort(frame, cameraMatrix, dist)
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, MY_DICT)
        
        if ids is not None and len(corners) > 0:
            center = (int(np.mean(corners[0][0][:, 0])), int(np.mean(corners[0][0][:, 1])))
            instructions[0] = 1.0  
            instructions[1] = findPhi(center)  

            instructions[2] = 1.0  
            instructions[3] = distance(corners, ids) or 0.0  

            with data_lock:
                latest_data["angle"] = instructions[1]
                latest_data["dist"] = instructions[3]
                latest_data["arrow"] = "N"  
        else:
            instructions = [0.0] * 6  

        send_instructions()
        cv2.imshow('Demo2', frame_undistorted)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
    endThread = True
