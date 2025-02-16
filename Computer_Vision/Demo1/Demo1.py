'''
*******************************************************************
* File Name         : Demo1.py

* Description       : Accurately measure angle of centroid of ArUco 
* marker relative to camera and report to LCD screen
*
* Supplementary File(s): sample.ino                  
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/10/2025    Kobe Prior and Blane Miller Created File
******************************************************************
Hardware Setup: <TODO>
Example Excecution: <TODO>
'''
import cv2
from cv2 import aruco
import numpy as np
import pickle
import sys

def find_phi(fov, object_pixel, image_width):
    """
    Calculate the angle (phi) of the object relative to the camera center.
    """
    half_fov = fov / 2
    center_pixel = image_width / 2
    pixel_ratio = (object_pixel - center_pixel) / center_pixel
    phi = half_fov * pixel_ratio
    return phi  # Returning in degrees (- means left relative to camera)

def load_calibration():
    """
    Loads camera calibration data from pickle files, handling potential errors.
    """
    try:
        with open("calibration.pkl", "rb") as f:
            camera_matrix, dist_coeffs = pickle.load(f)
        return camera_matrix, dist_coeffs
    except (FileNotFoundError, IOError) as e:
        print(f"Error loading calibration data: {e}")
        sys.exit(1)

def detect_aruco_live():
    """
    Continuously captures frames from the camera, detects ArUco markers, and calculates their angle.
    """
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Could not open camera.")
        sys.exit(1)
    
    fov = 68.5  # Field of view in degrees
    camera_matrix, dist_coeffs = load_calibration()
    
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Failed to capture image.")
            break
        
        image_width = frame.shape[1]  # Get image width dynamically
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        my_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        
        # Detect markers
        corners, ids, _ = aruco.detectMarkers(grey, my_dict, cameraMatrix=camera_matrix, distCoeff=dist_coeffs)
        overlay = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)
        overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor=4)
        
        if ids is not None:
            ids = ids.flatten()
            for (outline, marker_id) in zip(corners, ids):
                marker_corners = outline.reshape((4, 2))
                center_pixel_x = int(np.mean(marker_corners[:, 0]))
                center_pixel_y = int(np.mean(marker_corners[:, 1]))
                
                # Display marker ID and center position
                overlay = cv2.putText(overlay, str(marker_id),
                                      (int(marker_corners[0, 0]), int(marker_corners[0, 1]) - 15),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                overlay = cv2.putText(overlay, "+", (center_pixel_x, center_pixel_y),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                # Calculate angle
                phi = find_phi(fov, center_pixel_x, image_width)
                print(f'ArUco marker {marker_id} is {phi:.2f} degrees from camera center')
        
        cv2.imshow("Live Detection", overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_aruco_live()