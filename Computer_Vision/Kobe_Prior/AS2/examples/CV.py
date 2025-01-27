import numpy as np
import cv2 as cv

#img = cv.imread('black_man_window.jpg', 0)
#plt.imshow(img, cmap = 'gray', interpolation='bicubic')
#plt.xticks([]), plt.yticks([])
#plt.show();
#cv.waitKey(0)
#cv.destroyAllWindows()

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("cannot open")
    exit()
while True:
    #capture frame by frame
    ret, frame = cap.read()

    #if frame is read correctly ret is TRUE
    if not ret:
        print("cant' recieved frame (stream end?). Exiting ...")
        break
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.imshow('frame', gray)
    if cv.waitKey(1) == ord('q'):
        break
cap.release()
cv.destroyAllWindows()
