from time import sleep
import numpy as np
import cv2


fileName = input("File Name: ")
	
# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)
	
# Let the camera warmup
sleep(1)
	
# Get an image from the camera stream
ret, image = camera.read()
	
if not ret:
    print("Could not capture image from camera!")
    quit()
else:
    print("if you would like to save press the s key if you want to leave press esc")
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    edges = cv2.Canny(image, 100, 200)
    cv2.imshow('image', HSV)#show the image captured in black and white
    k = cv2.waitKey(0)
    #if the use types s save
    if k == 27:
        cv2.destroyAllWindows()
    elif k == ord('s'):
        try: 
            cv2.imwrite(fileName,image)
        except:
            print("could not write")
            pass
        cv2.destroyAllWindows()
		
		
	
