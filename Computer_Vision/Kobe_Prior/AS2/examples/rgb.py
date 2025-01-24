import cv2 as cv
import numpy as np

#imagebgr = cv.imread('color-wheel.png')
#y,x,c = imagebgr.shape
#myHSV = imagehsv[int(y/2), int(x/2)]
#print(myHSV)

slippers = cv.imread(ruby_slippers.jpg)
slippersHSV = cv.cvtCOLOR(slippers, cv.COLORBGR2HSV)
