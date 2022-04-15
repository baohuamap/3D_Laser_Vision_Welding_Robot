import cv2 as cv
import numpy as np
from pypylon import pylon
from MyLibrary import vision, yaskawa
import time

vis = vision()
img = cv.imread("D:\Code\Python_Code\WeldingRobot\VisionVer11\Mycamera\Laser\Triple Laser2.png",0)
# img = cv.imread("D:\Code\Python_Code\WeldingRobot\VisionVer11\Mycamera\Laser/14.jpg",0)
rows,cols = img.shape
med = cv.medianBlur(img, 9) 
thinned = cv.ximgproc.thinning(med)
cv.imshow("temp", thinned)

# rows,cols = img.shape
mainline_img = np.zeros((rows,cols))
roi = np.where(thinned.transpose() == 255)      #roi[0]: column, roi[1] row 
print(roi[0][0:50]) 
a = np.where(roi[0] == 269)
print(a)

previous = np.array([0,0])
current = np.array([0,0])
for i in range(300, cols - 300):
    a = np.where(roi[0] == i)   # find pixel which value is 255 in each col
    if len(a[0]) == 3:
        ave = (max(roi[1][a[0]]) + min(roi[1][a[0]]))/ 2
        average_index = np.abs(roi[1][a[0]] - ave).argmin()
        mainline_img[roi[1][a[0][1]],roi[0][a[0][1]]] = 255
        cv.circle(thinned, (roi[0][a[0][average_index]], roi[1][a[0][average_index]]), 5, 255, 1)
        previous = np.array([roi[0][a[0][average_index]],roi[1][a[0][average_index]]])

    elif len(a[0]) != 0:
        average_index = np.abs(roi[1][a[0]] - ave).argmin()
        current = np.array([roi[0][a[0][average_index]],roi[1][a[0][average_index]]])
        if np.linalg.norm(current[0:3] - previous[0:3]) < 10:
            cv.circle(thinned, (roi[0][a[0][average_index]],roi[1][a[0][average_index]]), 5, 255, 1)
            previous = current

cv.imshow("thinned", thinned)
cv.waitKey(0)
