import numpy as np
import cv2 as cv
import glob
from MyLibrary import vision

vis = vision()
intrinsic = vis.intrinsic
dist_coffs = vis.dist_coffs

objp = vis.CalibParams()
detector = vis.CalibParams()
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

R_gripper2base = []
t_gripper2base = []

posc1 = [59.088, -1637.8829999999998, -465.71099999999996, 148.25, 1.59, 23.97]
posc2 = [86.326, -1637.912, -465.638, 156.75, 11.219999999999999, 23.39]
posc3 = [86.351, -1637.9360000000001, -465.633, 144.0, 15.739999999999998, 39.98]
posc4 = [86.389, -1637.542, -364.153, 165.36, 29.64, 46.74]
posc5 = [86.36999999999999, -1637.197, -365.156, 176.91, 35.89, 88.33]
posc6 = [51.146, -1662.454, -366.09599999999995, 164.73, 19.810000000000002, 31.51]
posc7 = [62.29900000000001, -1516.107, -368.048, 125.66999999999999, 34.63, 12.93]
posc8 = [63.501, -1654.768, -367.889, 169.57, 22.08, 66.03]

R1, t1 = vis.posc2Rt(posc1)
R2, t2 = vis.posc2Rt(posc2)
R3, t3 = vis.posc2Rt(posc3)
R4, t4 = vis.posc2Rt(posc4)
R5, t5 = vis.posc2Rt(posc5)
R6, t6 = vis.posc2Rt(posc6)
R7, t7 = vis.posc2Rt(posc7)
R8, t8 = vis.posc2Rt(posc8)

R_gripper2base.append(R1)
t_gripper2base.append(t1)
R_gripper2base.append(R2)
t_gripper2base.append(t2)
R_gripper2base.append(R3)
t_gripper2base.append(t3)
R_gripper2base.append(R4)
t_gripper2base.append(t4)
R_gripper2base.append(R5)
t_gripper2base.append(t5)
R_gripper2base.append(R6)
t_gripper2base.append(t6)
R_gripper2base.append(R7)
t_gripper2base.append(t7)
R_gripper2base.append(R8)
t_gripper2base.append(t8)
R_target2cam = []
t_target2cam = []

images = glob.glob('Mycamera\HandeyeCalibration\*.jpg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    ret, corners = cv.findCirclesGrid(gray, (4,11),None,flags=(cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING), blobDetector=detector)
    if ret:
        corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        img = cv.drawChessboardCorners(img, (4,11), corners2,ret)
        retval, rvec, tvec	= cv.solvePnP(objp,corners2,intrinsic,dist_coffs)
        rotation_matrix = np.zeros(shape=(3,3))
        cv.Rodrigues(rvec, rotation_matrix)
        R_target2cam.append(rotation_matrix)
        t_target2cam.append(tvec)
        resized = cv.resize(img, (1080, 720), interpolation = cv.INTER_AREA)
        cv.imshow("pic: ", resized)
        cv.waitKey(0)

R_cam2gripper, t_cam2gripper = cv.calibrateHandEye(R_gripper2base ,t_gripper2base ,R_target2cam, t_target2cam)

print("rotation and translation matrix camera to welding tool:")
print("R", R_cam2gripper)
print("t", t_cam2gripper)
