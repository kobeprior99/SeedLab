import cv2
from cv2 import aruco
import numpy as np
import pickle
from time import sleep

# Load the camera calibration parameters
cameraMatrix = pickle.load(open("cameraMatrix.pkl", "rb"))
dist = pickle.load(open("dist.pkl", "rb"))

# Function to calculate the angle (phi) of the marker
def findPhi(fov, object_pixel, image_width):
    half_fov = fov // 2
    center_pixel = image_width // 2
    pixel_ratio = (object_pixel - center_pixel) / center_pixel
    phi = half_fov * pixel_ratio
    return phi

# Open the video stream (use 0 for default camera or set another number if using multiple cameras)
camera = cv2.VideoCapture(0)
sleep(0.5)

# ArUco dictionary and parameters
myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
fov = 68.5  # Field of view in degrees (adjust as needed)

# Initialize video capture and prepare for live stream
while True:
    ret, frame = camera.read()

    if not ret:
        print("Failed to grab frame.")
        break

    # Undistort the frame using the calibration parameters
    h, w = frame.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
    frame = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)

    # Convert the image to grayscale for ArUco marker detection
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = aruco.detectMarkers(grey, myDict)

    # Draw the detected markers on the frame
    overlay = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)
    overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor=(0, 255, 0))

    # If markers are detected, calculate and display the angle
    if ids is not None:
        ids = ids.flatten()
        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4, 2))
            center_pixel_x = int(np.mean(markerCorners[:, 0]))  # Get x-coordinate of marker center
            center_pixel_y = int(np.mean(markerCorners[:, 1]))  # Get y-coordinate of marker center

            # Calculate the angle from the center of the image
            phi = findPhi(fov, center_pixel_x, w)

            # Display the center point and the angle on the frame
            overlay = cv2.putText(overlay, f"Angle: {phi:.2f}°", (center_pixel_x + 10, center_pixel_y),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Display the center of the marker
            overlay = cv2.putText(overlay, "+", (center_pixel_x, center_pixel_y),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Show the processed video stream with ArUco markers and angle
    cv2.imshow("ArUco Angle Detection", overlay)

    # Check for key press to break the loop (press 'q' to quit)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the OpenCV windows
camera.release()
cv2.destroyAllWindows()
