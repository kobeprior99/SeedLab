
import cv2 as cv
import numpy as np
from time import sleep


# mask_green function that takes in an image and returns a mask for the green color
def mask_green(img):
    # [64, 140, 73] value using colorThresholdFinder.py from tutorial
    # green hue between 100 and 140 degrees
    # saturation    0-255
    # value         25-255
    lowergreen = np.array([50, 50, 50])
    uppergreen = np.array([80, 255, 255])
    # convert the image to hsv
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    #create a binary mask for the green color
    mask = cv.inRange(img_hsv, lowergreen, uppergreen)

    #apply morphological transformations to refine the mask
    #remove noise, kernal: rectangular 5x5
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((5,5),np.uint8))
    #fill in holes, kernal: rectangualr 5x5
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, np.ones((5,5),np.uint8))

    # check the mask to see if it worked
    # cv.imshow("mask", mask)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    return mask

#make a function that takes in an image and a mask and identifies green shapes
def id_green(img, mask):
    #find the contours in the mask
    contours,_ = cv.findContours(mask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    #draw the contours on the image if the area is greater than 100
    for contour in contours:
        if cv.contourArea(contour) > 400:
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv.putText(img, 'Green', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    return img

#experimentation with static image for image processing
# img = cv.imread("static_colors.png")
# cv.imshow("colors", img) 
# mask = mask_green(img)
#show the mask to check if it worked
# cv.imshow("mask", mask)
# cv.waitKey(0)
# cv.destroyAllWindows()    

# initialize camera
camera = cv.VideoCapture(0)
if not camera.isOpened():
    print("cannot open camera")
    exit()
while True: 
    ret, frame = camera.read()
    cv.imshow("green detection", id_green(frame, mask_green(frame)))
    key = cv.waitKey(1)
    if key == ord("q"):
        break

#turn off the camera and destroy all windows
camera.release()
cv.destroyAllWindows()
