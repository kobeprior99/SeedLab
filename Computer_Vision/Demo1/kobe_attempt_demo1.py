import cv2
from cv2 import aruco
import numpy as np
import pickle
import sys
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue

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
            newAngle = LCDqueue.get()
            try:
                lcd.clear()
                lcd.message = "Angle:\n" + str(newAngle)
            except Exception as e:
                print(f"Failed to update LCD: {e}")
        if endQueue:
            break
        

def find_phi(center,camera_matrix):
    """
    Calculate the angle (phi) of the object relative to the camera center.
    """
    # Get the optical center and focal lengths from the camera matrix
    fx = camera_matrix[0, 0]  # Focal length in x
    cx = camera_matrix[0, 2]  # Optical center x-coordinate
    
    # Get the center of the marker (center is the tuple (center_pixel_x, center_pixel_y))
    marker_center_x, _ = center
    #only looking for horizontal angle
    
    # Convert pixel coordinates to normalized camera coordinates
    delta_x = marker_center_x - cx #distance from marker center to camera center
    phi = np.arctan2(delta_x,fx)
    # Calculate the angle (phi) between the camera center and the marker center
    
    # Convert phi to degrees
    phi_degrees = np.degrees(phi)
    
    return round(phi_degrees,2)

    

def load_calibration():
    """
    Loads camera calibration data from pickle files, handling potential errors.
    """
    try:
        with open("calibration.pkl", "rb") as f:
            camera_matrix,dist_coeffs,rvecs, tvecs = pickle.load(f)
        return camera_matrix, dist_coeffs, rvecs, tvecs
    except (FileNotFoundError, IOError, pickle.UnpicklingError) as e:
        print(f"Error loading calibration data: {e}")
        sys.exit(1)

def detect_aruco_live():
    """
    Continuously captures frames from the camera, detects ArUco markers, and calculates their angle.
    """
    #start LCD thread
    LCDthread = threading.Thread(target = LCDdisplay, args=())
    LCDthread.start()

    oldAngle = 0
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Could not open camera.")
        sys.exit(1)
    
    camera_matrix, dist_coeffs, _, _ = load_calibration()
    
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Failed to capture image.")
            break
                
        # Undistort with Remapping
        h,  w = frame.shape[:2]
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 1, (w,h))
        mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, newCameraMatrix, (w,h), 5)
        undistorted_img = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)


        grey = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
        my_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        
        # Detect markers
        corners, ids, _ = aruco.detectMarkers(grey, my_dict)
        overlay = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)
        overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor=4)
        
        if ids is not None:
            ids = ids.flatten()
            for (outline, marker_id) in zip(corners, ids):
                marker_corners = outline.reshape((4, 2))
                center_pixel_x = int(np.mean(marker_corners[:, 0]))
                center_pixel_y = int(np.mean(marker_corners[:, 1]))
                center = (center_pixel_x,center_pixel_y)
                # Display marker ID and center position
                overlay = cv2.putText(overlay, str(marker_id),(int(marker_corners[0, 0]), int(marker_corners[0, 1]) - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.circle(overlay, center, 1, (255,0,0), 1)
                
                # Calculate angle
                
                newAngle = find_phi(center, camera_matrix)
                if round(oldAngle,2) != round(newAngle,2):
                    oldAngle = newAngle
                    LCDqueue.put(newAngle)
                # print(f'ArUco marker {marker_id} is {phi:.2f} degrees from camera center')
        
        cv2.imshow("Live Detection", overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_aruco_live()
    endQueue = True