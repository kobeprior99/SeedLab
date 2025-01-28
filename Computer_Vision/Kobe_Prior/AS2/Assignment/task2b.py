'''
*******************************************************************
* File Name         : task2b.py
* Description       : take a picture of colors.pdf to detect green shapes, then perform
morphological transformations to clean up the mask.
*                    
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 01/27/2025	Kobe Prior	Created File
*
******************************************************************
Hardware Setup: Power on the Pi -> either connect peripherials or connect via PiConnect or other VNC service 
Example Excecution: -> open terminal -> navigate to the directory where the task2b.py file is located using cd command ->
run the python file using 'python task1.py' command and point the camera towards the green image.
'''

import cv2 as cv
import numpy as np
from time import sleep


#make a function that takes in an image and returns a mask of the green colors
def mask_green(img):
    #bounds for the green color use colorThresholdFinder.py to find these values [64, 140, 73] was the exact 
    #so I gave buffer on each side for live camera
    lowergreen = np.array([36, 25, 25])
    uppergreen = np.array([70, 255, 255])
    # convert the image to hsv
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    #create a mask for the green color
    mask = cv.inRange(img_hsv, lowergreen, uppergreen)
    #apply morphological transformations to clean up the mask
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((5,5),np.uint8))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, np.ones((5,5),np.uint8))
    #modify the mask using morphological transformations
    return mask

def display_contours(img, mask):
    #find the contours in the mask
    contours,_ = cv.findContours(mask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    #draw the contours on the image
    cv.drawContours(img,contours,-1,(0,0,255),5)
    x, y, _, _ = cv.boundingRect(contours[0])
    cv.putText(img, 'Detected Green', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    #display the image with the contours
    cv.imshow("Contours",img)
    #wait for key press then close windows
    cv.waitKey(0)
    cv.destroyAllWindows()
    return contours

#experimentation with static image for image processing
# img = cv.imread("static_colors.png")
# cv.imshow("colors", img) 
# mask = mask_green(img)
#show the mask to check if it worked
# cv.imshow("mask", mask)
# cv.waitKey(0)
# cv.destroyAllWindows()    
# initialize camera
# contours = display_contours(img, mask)

camera = cv.VideoCapture(0)
sleep(.5)
ret, frame = camera.read()
#take a picture of the colors
if not ret:
    print("Could not capture image from camera!")
    quit()
else:
    print("if you would like to save press the s key if you want to leave press esc")
    mask = mask_green(frame)
    contours = display_contours(frame, mask)
#turn off the camera
camera.release()