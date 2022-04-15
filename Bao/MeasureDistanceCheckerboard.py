import numpy as np
import cv2 as cv
from pypylon import pylon
from MyLibrary import vision
T = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]])

vis = vision()
objp = vis.CalibParams()
intrinsic = vis.intrinsic
dist_coffs = vis.dist_coffs

camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.Open()

# demonstrate some feature access
new_width = camera.Width.GetValue() - camera.Width.GetInc()
if new_width >= camera.Width.GetMin():
    camera.Width.SetValue(new_width)

numberOfImagesToGrab = 1000
camera.StartGrabbingMax(numberOfImagesToGrab)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        # Access the image data.
        img = grabResult.Array
        ret, corners = cv.findChessboardCorners(img, (10,7),None)
        if ret:
            img = cv.drawChessboardCorners(img, (10,7), corners,ret)
            cv.cornerSubPix(img,corners,(11,11),(-1,-1),criteria)
            retval, rvec, tvec	= cv.solvePnP(objp,corners,intrinsic,dist_coffs)
            rotation_matrix = np.zeros(shape=(3,3))
            cv.Rodrigues(rvec, rotation_matrix)
            print("translation_matrix",tvec)
            print("rotation ",rotation_matrix)
            resize = cv.resize(img, (870,687), interpolation = cv.INTER_AREA)
        cv.imshow("baslerCam",resize )
        cv.waitKey(1)
