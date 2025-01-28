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
    #apply morphological transformations to refine the mask
    #remove noise
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((5,5),np.uint8))
    #fill in holes
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, np.ones((5,5),np.uint8))
    #modify the mask using morphological transformations

    # check the mask to see if it worked
    cv.imshow("mask", mask)
    cv.waitKey(0)
    cv.destroyAllWindows()
    return mask

def display_contours(img, mask):
    #find the contours in the mask
    contours,_ = cv.findContours(mask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    #draw the contours on the image if the area is greater than 100
    for contour in contours:
        if cv.contourArea(contour) > 400:
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv.putText(img, 'Detected Green', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    #display the image with the contours
    cv.imshow("Green detection",img)
    #if for video delete thse two lines
    if cv.waitKey(0) == ord('q'):
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
camera = cv.VideoCapture(0)
camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
if not camera.isOpened():
    print("cannot open camera")
    exit()
sleep(3)

ret, frame = camera.read()
if not ret:
    print("cannot capture frame from camera!")
    exit()
else:   
    try:
        # compute mask and display contours
        contours = display_contours(frame, mask_green(frame))
    except:
        print("No green shapes detected!")

#peform detection in video instead of image
# while True: 
#     ret, frame = camera.read()
#     #take a picture of the colors
#     if not ret:
#         print("Could not capture frame from camera!")
#         break
#     else:
#         try:
#             # compute mask and display contours
#             contours = display_contours(frame, mask_green(frame))
#             if cv.waitKey(1) == ord('q'):
#                 break
#         except:
#             print("No green shapes detected!")


#turn off the camera and destroy all windows
camera.release()
cv.destroyAllWindows()
