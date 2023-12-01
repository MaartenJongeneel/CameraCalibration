import numpy as np
import cv2 as cv
import glob
import csv
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
sizex = 4
sizey = 7
sizes = 51.4 #size of one square in mm
objp = np.zeros((sizex*sizey,3), np.float32)
objp[:,:2] = np.mgrid[0:sizey,0:sizex].T.reshape(-1,2)
objp=objp*sizes
print(objp)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.png')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (sizey,sizex), None)
    # If found, add object points, image points (after refining them)
    print("test")
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (sizey,sizex), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
cv.destroyAllWindows()

# open the file in the write mode
f = open('camera_poses.csv', 'w', newline='')
# create the csv writer
writer = csv.writer(f)
for ii in range(12):
    tvec = np.transpose(tvecs[ii])/1000
    rvec = np.transpose(rvecs[ii])
    row = np.append(tvec,rvec)
    writer.writerow(row)
# close the file
f.close()
