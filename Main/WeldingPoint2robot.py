import cv2 as cv
import numpy as np

from MyLib import Vision

vis = Vision()
intrinsic = vis.intrinsic
dist_coffs = vis.dist_coffs
eye2hand = vis.eye2hand
[a,b,c,d] = vis.plane
fx = intrinsic[0][0]
fy = intrinsic[1][1]
cx = intrinsic[0][2]
cy = intrinsic[1][2]    

u = 739.5
v = 525

# welding point to camera coxordinate
Zc = -d / (a/fx*(u-cx)+b/fy*(v-cy)+c)
weldpoint2camera = np.array([[Zc/fx*(u-cx)], [Zc/fy*(v-cy)], [Zc] , [1]])

# tool to robot coordinate
# pos1
tool2robot = vis.homogeneous([1018.740, -12.387, -477.997, 180.00, 0, 0])

# weldpoint to robot coordinate
weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)

print(weldpoint2robot)