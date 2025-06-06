'''
*******************************************************************
* File Name         : Demo1.py

* Description       : Accurately measure angle of centroid of ArUco 
* marker relative to camera and report to LCD screen
*
* Supplementary File(s): cam_cal.py used to generate intrinsic camera parameters stored in calibration.pkl file   
* blanetest.py, a draft file, found helpful results used in this code     
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/10/2025    Kobe Prior and Blane Miller Created File
* 02/16/2025    Kobe Prior and Blane Miller Made substantial progress: angle detection within reasonable error
******************************************************************
Hardware Setup: 
- Raspberry Pi
- Pi Camera
- I2C LCD Display
- ArUco markers 5x5 cm per instructions on assignment

Example Execution: 
- Ensure the camera is calibrated and calibration.pkl is available.
- Run the script using: python Demo1.py after navigating to the correct directory
- Place an ArUco marker in front of the camera to see the angle displayed on the LCD and the live video feed.

IDEAS TO MAKE MORE ACCURATE -> use full resolution of camera -> slower processing speed but higher precision ->requires we take photos again and regenerate calibration
'''

#import necessary libraries
import cv2
from cv2 import aruco
import numpy as np
import pickle  # Using pickle to load the calibration data
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue

# Load the camera calibration results
with open('calibration.pkl', 'rb') as f:
    cameraMatrix,dist,_,_ = pickle.load(f)
FX = cameraMatrix[0,0]
CX = cameraMatrix[0,2]


# LCD Dimensions and I2C
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
        # initialize LCD
        lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
        lcd.color = [100, 0, 0] #red
        lcd.clear()
    except Exception as e:
        print(f"LCD initialization failed: {e}")
        return

    while True:
        if not LCDqueue.empty():
            newAngle = LCDqueue.get_nowait()#get the latest angle without waiting
            while not LCDqueue.empty():
                LCDqueue.get_nowait()#clear the queue of intermediate angles
            try:
                lcd.clear()
                lcd.message = f"Angle:\n{newAngle:.2f}"#rounded to 2 decimal places
            except Exception as e:
                print(f"Failed to update LCD: {e}")
        if endQueue:
            lcd.clear()
            break


def find_center(corners):
    """
    Calculate the center coordinates of an ArUco marker.

    Args:
        corners (list): A list of corner points of the detected ArUco marker.

    Returns:
        tuple: The (x, y) coordinates of the marker's center.
    """
    for outline in corners:
        marker_corners = outline.reshape((4,2))
        
        # Compute the center of the marker
        center_x = int(np.mean(marker_corners[:, 0]))
        center_y = int(np.mean(marker_corners[:, 1]))

    return (center_x, center_y)

def findPhi(object_pixel):
    """
    Calculate the angle (phi) of an object relative to the camera's center.

    Args:
        object_pixel (int): The x-coordinate of the object's pixel position in the image.
    Returns:
        float: The calculated angle (phi) in degrees, rounded to two decimal places.
    """
    # Compute angle using arctan
    #return positive angle when marker is left of camera axis, negative when right
    phi = np.degrees(np.arctan((CX - object_pixel) / FX))
    
    return phi

def main():
    """
    Detects ArUco markers in the camera feed, calculates their angle relative to the camera's center, and updates the LCD display.

    This function initializes the camera feed and starts an infinite loop to continuously capture frames. 
    It undistorts each frame, converts it to grayscale, and detects ArUco markers. 
    For each detected marker, it calculates the angle relative to the camera's center and updates the LCD display if the angle has changed. 
    The loop breaks if the 'q' key is pressed.

    Exceptions:
        Prints error messages if camera initialization or frame capture fails.
    """

    # initialize angle
    oldAngle = 0.00

    #start LCD thread
    LCDthread = threading.Thread(target = LCDdisplay, args=())
    LCDthread.start()

    # Open the camera feed
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return


    # ArUco dictionary and parameters
    myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    #parameters = aruco.DetectorParameters_create()

    #start infinite loop press q to quit
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Undistort the frame using the camera matrix and distortion coefficients
        frame_undistorted = cv2.undistort(frame, cameraMatrix, dist)

        # Convert the image to grayscale then apply adaptive threshold that helps exemplify contours for aruco detection
        # gray = cv2.adaptiveThreshold(cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY), 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        #sharpening filter
        kernel = np.array([[0, -1, 0],  
                        [-1, 5,-1],  
                        [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)
        corners, _, _ = aruco.detectMarkers(gray, myDict)
        
        if len(corners) > 0:
            # Draw the detected markers
            frame_undistorted = aruco.drawDetectedMarkers(frame_undistorted, corners, borderColor=(0,255,0))

            center = find_center(corners)

            # Mark the center of the marker on the frame (mostly for debuging purposes)
            cv2.circle(frame_undistorted, center, 3, (0, 255, 0), -1)

            # Calculate the angle of the marker relative to the camera's center
            newAngle = findPhi(center[0])

            ANGLE_THRESHOLD = 0.01
            if(abs(newAngle-oldAngle) >= ANGLE_THRESHOLD):
                oldAngle = newAngle

                #queue only keeps most recent angle
                if not LCDqueue.empty():
                    LCDqueue.get_nowait() #remove old value immediately
                LCDqueue.put(newAngle)


            # Display the angle text on the frame above arcuo marker
            cv2.putText(frame_undistorted, f"{newAngle:.2f} degrees", (center[0] + 10, center[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Show the output frame with the detected marker and angle
        cv2.imshow('ArUco Marker Detection', frame_undistorted)

        # Break the loop on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


#run the program
if __name__ == "__main__":
    main()
    endQueue = True #after detect_marker_and_angle finishes end LCD thread
