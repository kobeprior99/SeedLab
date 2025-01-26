import cv2
from cv2 import aruco
import numpy as np
from time import sleep

myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
myArucoImg = aruco.generateImageMarker(myDict, 0, 400)
# cv2.imshow("Aruco", myArucoImg)
# cv2.imwrite("aruco.png", myArucoImg)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
camera = cv2.VideoCapture(0)
sleep(.5)
ret, frame = camera.read()
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
cv2.imshow("overlay",overlay)
cv2.waitKey(0)
cv2.destroyAllWindows()
camera.release()