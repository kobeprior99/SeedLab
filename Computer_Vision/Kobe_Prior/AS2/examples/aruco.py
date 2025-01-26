import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep

myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
myArucoImg = aruco.generateImageMarker(myDict, 0, 400)
# cv.imshow("Aruco", myArucoImg)
# cv.imwrite("aruco.png", myArucoImg)
# cv.waitKey(0)
# cv.destroyAllWindows()
camera = cv.VideoCapture(0)
sleep(.5)
ret, frame = camera.read()
grey = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #make the image greyscale for aruco detection
# cv.imshow("overlay", grey)
corners, ids, rejected =aruco.detectMarkers(grey, myDict)

overlay = cv.cvtColor(grey, cv.COLOR_GRAY2RGB)
overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor = 4)
if not ids is None:
    ids = ids.flatten()
    for (outline, id) in zip(corners, ids):
        markerCorners = outline.reshape((4,2))
        overlay = cv.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)
cv.imshow("overlay",overlay)
cv.waitKey(0)
cv.destroyAllWindows()
camera.release()