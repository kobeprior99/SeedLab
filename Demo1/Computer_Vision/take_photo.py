#credit: https://github.com/niconielsen32/CameraCalibration/blob/main/getImages.py
"""
Open Source code courtesy of niconielsen32

This script captures images from the default camera and saves them when the 's' key is pressed.
The captured images are saved in the 'images' directory with filenames 'img0.png', 'img1.png', etc.
Press the 'Esc' key to exit the script.
"""

import cv2

cap = cv2.VideoCapture(0)

num = 0

while cap.isOpened():

    success, img = cap.read()
    if not success: 
        break
    
    k = cv2.waitKey(5)

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('images/img' + str(num) + '.png', img)
        print("image saved!")
        num += 1

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()