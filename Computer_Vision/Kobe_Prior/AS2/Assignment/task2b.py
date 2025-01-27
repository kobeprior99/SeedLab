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
#make a function that takes in an image and returns the processed image
def find_green_image(image):
    


#experimentation with static image for image processing


# initialize camera
camera = cv.VideoCapture(0)
sleep(.5)
ret, frame = camera.read()


#turn off the camera
camera.release()