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

slippersBox=slippers.copy()
slipperCenters = np.empty((0,2))
slipperAreas = np.empty((0))

for index,cnt in enumerate(contours_red):
    contour_area = cv.contourArea(cnt)
    if contour_area > 300:
        x, y, w, h = cv.boundingRect(cnt)
        center = int(x+w/2),int(y+h/2)
        slipperAreas = np.append(slipperAreas,contour_area)
        slipperCenters = np.vstack((slipperCenters,center))
        cv.rectangle(slippersBox, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv.putText(slippersBox, 'Red', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        cv.putText(slippersBox, '+', center, cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
cv.imshow("Big Contours",slippersBox)
cv.waitKey(0)
cv.destroyAllWindows()    
print((slipperCenters,slipperAreas))
# cv.imshow("Result", result) 
# cv.waitKey(0)
# cv.destroyAllWindows()