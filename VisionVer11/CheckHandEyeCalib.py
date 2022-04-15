import numpy as np
import cv2 as cv
from MyLibrary import vision

def Rt2homo(R,t):
    transformation = np.zeros((4,4),float)
    for i in range(3):
        transformation[i][3] = t[i][0]
        for j in range(3):
            transformation[i][j] = R[i][j]
    transformation[3][3] = 1
    return transformation

vis = vision()
intrinsic = vis.intrinsic
dist_coffs= vis.dist_coffs
objp = vis.CalibParams()
print(objp)
Tchecker2tool= np.array([[1,0,0,-33.179],
                        [0,1,0,19.56],
                        [0,0,1,79.602],
                        [0,0,0,1]])

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

img = cv.imread("D:/Desktop/Mycamera/HandeyeCalibration/1.png")
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
ret, corners = cv.findChessboardCorners(gray, (10,7),None)
if ret:
    corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    retval, rvec, tvec	= cv.solvePnP(objp,corners2,intrinsic,dist_coffs)
    rotation_matrix = np.zeros(shape=(3,3))
    cv.Rodrigues(rvec, rotation_matrix)
    Tcam2tool = Tchecker2tool.dot(np.linalg.inv(Rt2homo(rotation_matrix,tvec)))
    print("Tcam2tool",Tcam2tool)
    img = cv.drawChessboardCorners(img, (10,7), corners2,ret)
    cv.circle(img, (int(corners2[0][0][0]),int(corners2[0][0][1])), 20, [255,0,0], 10)
    resized = cv.resize(img, (1080, 720), interpolation = cv.INTER_AREA)
    cv.imshow("baslerCam",resized)
    cv.waitKey(0)
 