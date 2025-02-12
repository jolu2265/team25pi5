import numpy as np
import cv2 as cv
import glob
from picamera2 import Picamera2

import time

start_time = time.time()
end_time = start_time + 5

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('calib.png')

assert images != 'calib.png', 'Calibration Image Not Found'

picam = Picamera2()
picam.configure(picam.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)}))
picam.start()

# for fname in images:
# while True:
while time.time() < end_time:
    frame = picam.capture_array()
    
    # img = cv.imread(fname)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(frame, (7,6), corners2, ret)
        # cv.imshow('img', img)
        cv.imshow('frame', frame)
        cv.waitKey(1) # was 500
        
cv.destroyAllWindows()
picam.close()        
        
ret2, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print(mtx)
print("dist")
print(dist)


