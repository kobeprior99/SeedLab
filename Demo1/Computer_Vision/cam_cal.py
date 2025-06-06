'''
*******************************************************************
* File Name         : cam_cal.py

* Description       : develop camera parameters to counteract distortion and store them in a serialized pickle file to be used in later programs
*        
* Supplementary files: take_photo.py generates photos of checkerboard that get stored in images folder    
* References:   https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
* https://github.com/niconielsen32/CameraCalibration/blob/main/calibration.py
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/12/2025    Kobe Prior and Blane Miller Created File
* 02/16/2025    Kobe Prior and Blane Miller ran script and generated further documentation
******************************************************************
Example Excecution: Remotely Connect to Raspberry Pi, 
navigate to the directory containing images folder and this script,
on the raspberry pi run this script using python cam_cal.py, note that mean error calculation takes some time.
'''

# import necessary libraries
import numpy as np
import cv2 as cv
import glob
import pickle



################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (9,6)
frameSize = (640,480)



# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 23
objp = objp * size_of_chessboard_squares_mm


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


images = glob.glob('images/*.png')

for image in images:

    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

    # If found, add object points, image points (after refining them)
    if ret == True:

        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(10)


cv.destroyAllWindows()




############## CALIBRATION #######################################################

ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

# Save the camera calibration result for later use (we won't worry about rvecs / tvecs)

pickle.dump((cameraMatrix, dist, rvecs, tvecs), open( "calibration.pkl", "wb" ))



############## UNDISTORTION #####################################################
# # example
# img = cv.imread('images/img5.png')
# h,  w = img.shape[:2]
# newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))



# # Undistort
# dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

# # crop the image
# x, y, w, h = roi
# dst = dst[y:y+h, x:x+w]
# cv.imwrite('caliResult1.png', dst)


# Reprojection Error
#something under .1 reprojection total error is accpetable, in this case we got something like 0.04.
mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print( "total error: {}".format(mean_error/len(objpoints)) )