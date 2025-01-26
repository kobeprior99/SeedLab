import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep

myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
myArucoImg = aruco.generateImageMarker(myDict, 0, 400)
cv.imshow("Aruco", myArucoImg)
cv.imwrite("aruco.png", myArucoImg)
cv.waitKey(0)
cv.destroyAllWindows()