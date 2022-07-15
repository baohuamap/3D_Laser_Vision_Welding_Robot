import cv2 as cv
import numpy as np

from MyLibrary import vision

vis = vision()
intrinsic = vis.intrinsic
dist_coffs = vis.dist_coffs
eye2hand = vis.eye2hand
[a,b,c,d] = vis.plane
fx = intrinsic[0][0]
fy = intrinsic[1][1]
cx = intrinsic[0][2]
cy = intrinsic[1][2]    

u = 761.56647
v = 640.3359
# welding point to camera coxordinate
Zc = -d / (a/fx*(u-cx)+b/fy*(v-cy)+c)
weldpoint2camera = np.array([[Zc/fx*(u-cx)], [Zc/fy*(v-cy)], [Zc] , [1]])

# tool to robot coordinate
# pos1
tool2robot = vis.homogeneous([182.658, -1338.458, -514.513, 137.29, 28.47, 34.83])

# weldpoint to robot coordinate
weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)

print(weldpoint2robot)