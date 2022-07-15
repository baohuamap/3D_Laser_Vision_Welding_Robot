import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from MyLibrary import vision

vis = vision()
intrinsic = vis.intrinsic
dist_coffs= vis.intrinsic
eye2hand = vis.eye2hand
[a,b,c,d] = vis.plane
fx = intrinsic[0][0]
fy = intrinsic[1][1]
cx = intrinsic[0][2]
cy = intrinsic[1][2]

point3D = []

# position 1_______________________________________________________
pos1 = cv.imread("D:\Code\Python_Code\WeldingRobot\VisionVer11\Mycamera\Position/1.png",0)
pos1 = vis.Preprocessing(pos1)
centerpos1 = vis.LaserCenter(pos1)
laserpos1 = np.where(centerpos1 == 255)

# mapping point to robot coordinate _______________________________
tool2robot = vis.homogeneous([216.58200000000002, -1695.762, -353.31, 153.18, -56.949999999999996, -128.45000000000002])
for i in range(laserpos1[0].size):
    Zc = -d / (a/fx*(laserpos1[1][i]-cx)+b/fy*(laserpos1[0][i]-cy)+c)
    weldpoint2camera = np.array([[Zc/fx*(laserpos1[1][i]-cx)], [Zc/fy*(laserpos1[0][i]-cy)], [Zc], [1]])
    weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)
    point3D.append(weldpoint2robot)
del weldpoint2camera
del weldpoint2robot
del tool2robot

# # position 2_______________________________________________________
# pos2 = cv.imread("D:/Desktop/Mycamera/Position/2.jpg",0)
# rows,cols = pos2.shape
# pos2 = vis.Preprocessing(pos2)
# centerpos2 = vis.LaserCenter(pos2)
# laserpos2 = np.where(centerpos2 == 255)

# # mapping point to robot coordinate _______________________________
# R_robot = vis.RPY2mtrx(110.67, 27.58, 75.79)
# t_robot = np.array([[607.991],[ -1003.107],[ -402.780]])
# tool2robot = vis.homogeneous(R_robot, t_robot)
# del R_robot
# del t_robot
# for i in range(laserpos2[0].size):
#     Zc = -d / (a/fx*(laserpos2[1][i]-cx)+b/fy*(laserpos2[0][i]-cy)+c)
#     weldpoint2camera = np.array([[Zc/fx*(laserpos2[1][i]-cx)], [Zc/fy*(laserpos2[0][i]-cy)], [Zc], [1]])
#     weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)
#     point3D.append(weldpoint2robot)

# del weldpoint2camera
# del weldpoint2robot
# del tool2robot

# # position 3_______________________________________________________
# pos3 = cv.imread("D:/Desktop/Mycamera/Position/3.jpg",0)
# rows,cols = pos3.shape
# pos3 = vis.Preprocessing(pos3)
# centerpos3 = vis.LaserCenter(pos3)
# laserpos3 = np.where(centerpos3 == 255)

# # mapping point to robot coordinate _______________________________
# R_robot = vis.RPY2mtrx(110.67, 27.58, 75.79)
# t_robot = np.array([[559.939],[ -1003.143],[ -402.843]])
# tool2robot = vis.homogeneous(R_robot, t_robot)
# del R_robot
# del t_robot
# for i in range(laserpos3[0].size):
#     Zc = -d / (a/fx*(laserpos3[1][i]-cx)+b/fy*(laserpos3[0][i]-cy)+c)
#     weldpoint2camera = np.array([[Zc/fx*(laserpos3[1][i]-cx)], [Zc/fy*(laserpos3[0][i]-cy)], [Zc], [1]])
#     weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)
#     point3D.append(weldpoint2robot)

# del weldpoint2camera
# del weldpoint2robot
# del tool2robot

# # position 4_______________________________________________________
# pos4 = cv.imread("D:/Desktop/Mycamera/Position/4.jpg",0)
# rows,cols = pos4.shape
# pos4 = vis.Preprocessing(pos4)
# centerpos4 = vis.LaserCenter(pos4)
# laserpos4 = np.where(centerpos4 == 255)

# # mapping point to robot coordinate _______________________________
# R_robot = vis.RPY2mtrx(110.67, 27.58, 75.79)
# t_robot = np.array([[660.759],[ -1003.123],[ -390.039]])
# tool2robot = vis.homogeneous(R_robot, t_robot)
# del R_robot
# del t_robot
# for i in range(laserpos4[0].size):
#     Zc = -d / (a/fx*(laserpos4[1][i]-cx)+b/fy*(laserpos4[0][i]-cy)+c)
#     weldpoint2camera = np.array([[Zc/fx*(laserpos4[1][i]-cx)], [Zc/fy*(laserpos4[0][i]-cy)], [Zc], [1]])
#     weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)
#     point3D.append(weldpoint2robot)

# del weldpoint2camera
# del weldpoint2robot
# del tool2robot

# plot point in laser plane________________________________________
X = np.array([ ])
Y = np.array([ ])
Z = np.array([ ])
for i in range(len(point3D)):
    X = np.insert(X, i, point3D[i][0][0])
    Y = np.insert(Y, i, point3D[i][1][0])
    Z = np.insert(Z, i, point3D[i][2][0])
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")
ax.scatter(X, Y, Z, c='r' , marker = '.')
Ox = np.array([0])
Oy = np.array([0])
Oz = np.array([0])
ax.scatter(Ox, Oy, Oz, c='g' , marker = 'o')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim3d(-1000,1000)
ax.set_ylim3d(-1000,1000)
ax.set_zlim3d(-1000,1000)
plt.show()
resized_pos1 = cv.resize(centerpos1, (1080, 720), interpolation = cv.INTER_AREA)
cv.imshow("1", resized_pos1)
cv.waitKey(0)