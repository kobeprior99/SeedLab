import cv2 as cv
import numpy as np

#imagebgr = cv.imread('color-wheel.png')
#y,x,c = imagebgr.shape
#myHSV = imagehsv[int(y/2), int(x/2)]
#print(myHSV)

slippers = cv.imread("ruby_slippers.jpg")
slippersHSV = cv.cvtColor(slippers, cv.COLOR_BGR2HSV)
# note that Hue in cv is 0-180, not 0-360 so divide by 2 from actual angle
# [[ 11 222 246] upper found using colorThresholdFinder.py
#  [  0 191 125]] lower
upperred = np.array([16, 255, 198])
lowerred = np.array([1, 181, 63])
mask = cv.inRange(slippersHSV, lowerred, upperred) 
result = cv.bitwise_and(slippers, slippers, mask=mask) 
contour_red_visualize = slippers.copy()
contours_red,_ = cv.findContours(mask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
cv.drawContours(contour_red_visualize,contours_red,-1,(255,0,0),3)
cv.imshow("Contours",contour_red_visualize)
cv.waitKey(0)
cv.destroyAllWindows()
# cv.imshow("Result", result) 
# cv.waitKey(0)
# cv.destroyAllWindows()