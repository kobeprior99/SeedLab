import cv2 as cv
import numpy as np

#imagebgr = cv.imread('color-wheel.png')
#y,x,c = imagebgr.shape
#myHSV = imagehsv[int(y/2), int(x/2)]
#print(myHSV)

slippers = cv.imread("ruby_slippers.jpg")
slippersHSV = cv.cvtCOLOR(slippers, cv.COLORBGR2HSV)
# note that Hue in cv is 0-180, not 0-360 so divide by 2 from actual angle
# [[ 11 222 246] upper found using colorThresholdFinder.py
#  [  0 191 125]] lower
upperred = np.array([11, 222, 246])
lowerred = np.array([0, 191, 125])
mask = cv.inRange(slippersHSV, lowerred, upperred)