import cv2
from cv2 import aruco
import numpy as np
import pickle  

# Load camera calibration results
with open('calibration.pkl', 'rb') as f:
    cameraMatrix, dist, _, _ = pickle.load(f)

MARKER_WIDTH_IRL = 2  # inches (real-world size)
myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

def find_center(corners):
    """ Calculate the center of the detected ArUco marker. """
    marker_corners = corners.reshape((4, 2))
    center_x = int(np.mean(marker_corners[:, 0]))
    center_y = int(np.mean(marker_corners[:, 1]))
    return (center_x, center_y)

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Undistort image to remove lens distortion effects
    frame_undistorted = cv2.undistort(frame, cameraMatrix, dist)
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = aruco.detectMarkers(gray, myDict)
    
    if ids is not None:  # If markers are detected
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_WIDTH_IRL, cameraMatrix, dist)
        
        for i in range(len(ids)):
            # Extract translation vector (tvec) to get distance
            distance_found = tvecs[i][0][2]  # Z-distance from camera to marker

            # Get center of marker
            center = find_center(corners[i])

            # Display distance on the image
            cv2.putText(frame_undistorted, f"{distance_found:.2f} inches", 
                        (center[0] + 10, center[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.6, (0, 0, 255), 2)

            print(f"Marker ID {ids[i][0]}: {distance_found:.2f} inches")

    cv2.imshow("distance", frame_undistorted)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
