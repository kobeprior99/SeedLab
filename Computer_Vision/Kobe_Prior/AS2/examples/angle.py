import cv2
from cv2 import aruco
import numpy as np
from time import sleep

def findPhi(fov, object_pixel, image_width):
    # fov is the field of view of the camera in degrees
    # object_pixel is the x coordinate of the object relative to the center of the image
    # image_width is the width of the image in pixels

    half_fov = fov / 2;
    center_pixel = image_width / 2;
    pixel_ratio = abs(object_pixel - center_pixel) / center_pixel;
    phi = half_fov * pixel_ratio;

    return phi;

camera = cv2.VideoCapture(0)
sleep(.5)
ret, frame = camera.read()

myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
fov =68.5 #in degrees found from website
grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #make the image greyscale for aruco detection
# cv2.imshow("overlay", grey)

corners, ids, rejected = aruco.detectMarkers(grey, myDict)
overlay = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)
overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor = 4)
if not ids is None:
    ids = ids.flatten()
    for (outline, id) in zip(corners, ids):
        markerCorners = outline.reshape((4,2))
        overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)
center_pixel_x = corners[0][0][0][0]
phi = findPhi(fov, center_pixel_x, 640)
cv2.imshow("overlay",overlay)
cv2.waitKey(0)
cv2.destroyAllWindows()
camera.release()
print(phi)