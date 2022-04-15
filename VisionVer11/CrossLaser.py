import cv2 as cv
import numpy as np
from pypylon import pylon
from MyLibrary import vision, yaskawa
import time

# a = np.arange(25).reshape(5,5)
# print(a)
# print(np.where(a>5))

vis = vision()
img = cv.imread("D:\Code\Python_Code\WeldingRobot\VisionVer11\Mycamera\Laser/CrossLaser4.png",0)
img = cv.resize(img, (1080, 720), interpolation = cv.INTER_AREA)
rows,cols = img.shape
med = cv.medianBlur(img, 9)
thinned = cv.ximgproc.thinning(med)
cv.imshow("temp", thinned)
rows,cols = img.shape
mainline_img = np.zeros((rows,cols))
roi = np.where(thinned.transpose() == 255)      #roi[0]: column, roi[1] row 

previous = np.array([0,0])
current = np.array([0,0])
for i in range(0, cols - 0):
    a = np.where(roi[0] == i)   # find pixel which value is 255 in each col
    if len(a[0]) == 1:
        cv.circle(thinned, (roi[0][a[0][0]], roi[1][a[0][0]]), 5, 255, 1)
        colminval = roi[1][a[0][0]]
    elif len(a[0]) > 1:
        average_index = np.abs(roi[1][a[0]] - colminval).argmin()
        cv.circle(thinned, (roi[0][a[0][average_index]], roi[1][a[0][average_index]]), 5, 255, 1)
        colminval = roi[1][a[0][average_index]]

cv.imshow("thinned", thinned)
cv.waitKey(0)
