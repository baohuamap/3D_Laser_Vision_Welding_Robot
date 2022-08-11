# Development of a real-time laser-based machine vision system to monitor and control welding processes 
# by Wei Huang & Radovan Kovacevic
import cv2 as cv
import numpy as np
from MyLibrary import vision
import time
vis = vision()
t = time.time()
img = cv.imread("D:/Code/Welding_Robot/3D_Laser_Vision_Welding_Robot/MyCamera/Laser_stripes/laser6.jpg",0)
rows,cols = img.shape
print(rows, cols)
img = vis.Preprocessing(img)
CD = np.zeros((rows,cols))
temp = np.uint8(vis.LaserCenter(img))
print("preprocessing time:", time.time() - t)
# central difference calculate
ran = 1
lefval = 0
rightval = 0
for y in range(500, rows - 500, 1):
    for x in range(ran, cols - ran, 1):
        if temp[y][x] == 255:
            for k in range(1, rows - 1, 1):
                if temp[k][x+ran] >= rightval:
                    rightval = temp[k][x+ran]
                    y1 = k
            for k in range(1, rows - 1, 1):
                if temp[k][x-ran] >= lefval:
                    lefval = temp[k][x-ran]
                    y2 = k
            CD[y][x] = (y1 - y2)/2
            lefval = 0
            rightval = 0
            y1 = 0
            y2 = 0
print("CD calculate time: ", time.time() - t )
# feature extraction base on CD value
# 1st feature and 2nd feature which is maximum CD and minimun CD
maxval = 0
feature1pos = np.array([0, 0])
minval = 0
feature2pos = np.array([0, 0])
for y in range(100, rows - 100, 1):
    for x in range(1, cols - 1, 1):
        if CD[y][x]>maxval:
            maxval = CD[y][x]
            feature1pos[1] = y
            feature1pos[0] = x
        if CD[y][x]<minval:
            minval = CD[y][x]
            feature2pos[1] = y
            feature2pos[0] = x
cv.circle(temp, (feature1pos[0],feature1pos[1]), 10, 255, 5) 
cv.circle(temp, (feature2pos[0],feature2pos[1]), 10, 255, 5)
resized = cv.resize(temp, (1080, 720), interpolation = cv.INTER_AREA)
cv.imshow("result1",resized)
print("total time:", time.time() - t)
cv.waitKey(0)
