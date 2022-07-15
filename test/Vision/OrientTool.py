import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import MyLibrary as my
from math import asin,pi,cos,sqrt

def axis2mtrx(vectorX, vectorY, vectorZ):
    unitX = (vectorX/np.linalg.norm(vectorX)).reshape(3,1)
    unity = (vectorY/np.linalg.norm(vectorY)).reshape(3,1)
    unitz = (vectorZ/np.linalg.norm(vectorZ)).reshape(3,1)
    mtrx =  np.concatenate((unitX, unity, unitz), axis=1)
    return mtrx

def coordinate(variable, position, rotation):
    xvecx = position[0][0]
    yvecx = position[1][0]
    zvecx = position[2][0]
    uvecx = rotation[0][0]
    vvecx = rotation[1][0]
    wvecx = rotation[2][0]
    xvecy = position[0][0]
    yvecy = position[1][0]
    zvecy = position[2][0]
    uvecy = rotation[0][1]
    vvecy = rotation[1][1]
    wvecy = rotation[2][1]
    xvecz = position[0][0]
    yvecz = position[1][0]
    zvecz = position[2][0]
    uvecz = rotation[0][2]
    vvecz = rotation[1][2]
    wvecz = rotation[2][2]
    return variable.quiver(xvecx, yvecx, zvecx, uvecx, vvecx, wvecx
    ,color = 'r', length=500, normalize=True),variable.quiver(xvecy, yvecy, zvecy, uvecy, vvecy, wvecy,
     length=500,color = 'g', normalize=True),variable.quiver(xvecz, yvecz, zvecz, uvecz, vvecz, wvecz,
     length=500, normalize=True)

intrinsic,dist_coffs,eye2hand,[a,b,c,d] = my.parameters()
fx = intrinsic[0][0]
fy = intrinsic[1][1]
cx = intrinsic[0][2]
cy = intrinsic[1][2]

plane1 = []
plane2 = []
point3D = []
# Position 1__________________________________________________
pos1 = cv.imread("D:/Desktop/Mycamera/Position/1.jpg",0)
rows,cols = pos1.shape
pre_pos1 = my.Preprocessing(pos1)
centerpos1 = my.LaserCenter(pre_pos1)
feature1 = my.FeatureExtraction(centerpos1)

laserpos1plane1 = np.where(centerpos1[:,int(feature1[0])-500:int(feature1[0])] == 255)
laserpos1plane2 = np.where(centerpos1[:,int(feature1[0]):int(feature1[0])+500] == 255)
R_robot = my.RPY2mtrx(175.71, -48.19, -58.41)
t_robot = np.array([[143.433],[-1715.335],[-451.719]])
tool2robot = my.homogeneous(R_robot, t_robot)
del R_robot
del t_robot

for i in range(laserpos1plane1[0].size):
    Zc = -d / (a/fx*(laserpos1plane1[1][i] + int(feature1[0])-500 -cx)+b/fy*(laserpos1plane1[0][i]-cy)+c)
    weldpoint2camera = np.array([[Zc/fx*(laserpos1plane1[1][i] + int(feature1[0])-500-cx)], [Zc/fy*(laserpos1plane1[0][i]-cy)], [Zc], [1]])
    weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)
    plane1.append(weldpoint2robot)
    point3D.append(weldpoint2robot)

for i in range(laserpos1plane2[0].size):
    Zc = -d / (a/fx*(laserpos1plane2[1][i]+feature1[0]-cx)+b/fy*(laserpos1plane2[0][i]-cy)+c)
    weldpoint2camera = np.array([[Zc/fx*(laserpos1plane2[1][i]+feature1[0]-cx)], [Zc/fy*(laserpos1plane2[0][i]-cy)], [Zc], [1]])
    weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)
    plane2.append(weldpoint2robot)
    point3D.append(weldpoint2robot)


# Position 2__________________________________________________
pos2 = cv.imread("D:/Desktop/Mycamera/Position/2.jpg",0)
rows,cols = pos2.shape
pre_pos2 = my.Preprocessing(pos2)
centerpos2 = my.LaserCenter(pre_pos2)
feature2 = my.FeatureExtraction(centerpos2)

laserpos2plane1 = np.where(centerpos2[:,int(feature2[0])-500:int(feature2[0])] == 255)
laserpos2plane2 = np.where(centerpos2[:,int(feature2[0]):int(feature2[0])+500] == 255)

R_robot = my.RPY2mtrx(175.69, -48.2, -58.39)
t_robot = np.array([[1.008],[ -1715.396],[ -451.752]])
tool2robot = my.homogeneous(R_robot, t_robot)
del R_robot
del t_robot

for i in range(laserpos2plane1[0].size):
    Zc = -d / (a/fx*(laserpos2plane1[1][i]+ int(feature2[0])-500 -cx)+b/fy*(laserpos2plane1[0][i]-cy)+c)
    weldpoint2camera = np.array([[Zc/fx*(laserpos2plane1[1][i] + int(feature2[0])-500 -cx)], [Zc/fy*(laserpos2plane1[0][i]-cy)], [Zc], [1]])
    weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)
    plane1.append(weldpoint2robot)
    point3D.append(weldpoint2robot)

for i in range(laserpos2plane2[0].size):
    Zc = -d / (a/fx*(laserpos2plane2[1][i]+feature2[0]-cx)+b/fy*(laserpos2plane2[0][i]-cy)+c)
    weldpoint2camera = np.array([[Zc/fx*(laserpos2plane2[1][i]+feature2[0]-cx)], [Zc/fy*(laserpos2plane2[0][i]-cy)], [Zc], [1]])
    weldpoint2robot = tool2robot.dot(eye2hand).dot(weldpoint2camera)
    plane2.append(weldpoint2robot)
    point3D.append(weldpoint2robot)
    
# plot point in laser plane________________________________________
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")
X = np.array([ ])
Y = np.array([ ])
Z = np.array([ ])
for i in range(len(point3D)):
    X = np.insert(X, i, point3D[i][0][0])
    Y = np.insert(Y, i, point3D[i][1][0])
    Z = np.insert(Z, i, point3D[i][2][0])
ax = fig.add_subplot(111,projection="3d")
ax.scatter(X, Y, Z, c='r' , marker = '.')
del X
del Y
del Z
x = np.linspace(-1000,1000,10)
y = np.linspace(-2000,-1000,10)
X,Y = np.meshgrid(x,y)
[a1, b1, d1] = my.PlaneFitting(plane1)
[a2, b2, d2] = my.PlaneFitting(plane2)

nPlane1 = np.array([a1,b1,1], dtype=float)  
nPlane2 = np.array([a2,b2,1], dtype=float)  
den1 = sqrt(a1**2 + b1**2 + 1)
den2 = sqrt(a2**2 + b2**2 + 1)
BisectorPlane = np.array([(a1*den2 - a2*den1),(b1*den2 - b2*den1),( den1 - den2),( d1*den2 - d2*den1)],dtype=float)
del den1
del den2

# calculate x axis of torch
nBisector = BisectorPlane[0:3]/np.linalg.norm(BisectorPlane[0:3])

Z1=a1*X + b1*Y + d1
Z2=a2*X + b2*Y + d2
Z3 = -(BisectorPlane[0]*X + BisectorPlane[1]*Y + BisectorPlane[3])/BisectorPlane[2]

# calculate y axis of torch
WeldSeam = np.cross(nPlane1, nPlane2)
WeldSeam = WeldSeam/np.linalg.norm(WeldSeam)

# calculate z axis of torch
Torch = np.cross(nBisector, WeldSeam)
coordinate(ax,[[3.27701308e+02],[-1.83306841e+03],[-5.12118961e+02]], axis2mtrx(nBisector,WeldSeam,Torch))
coordinate(ax,[[0],[0],[0]],[[1,0,0],[0,1,0],[0,0,1]])

print(my.mtrx2RPY(axis2mtrx(nBisector,WeldSeam,Torch)))
# draw the plane of welding point
# surf = ax.plot_surface(X, Y, Z1)
# surf = ax.plot_surface(X, Y, Z2)
# surf = ax.plot_surface(X, Y, Z3)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim3d(-1200,1000)
ax.set_ylim3d(-1200,1000)
ax.set_zlim3d(-1200,1000)
plt.show()
