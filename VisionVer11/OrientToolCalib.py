import numpy as np
import matplotlib.pyplot as plt
from MyLibrary import vision

def mtrx2RPY(mtrx):
    from math import atan2,sqrt,pi
    Rz=atan2(mtrx[1,0],mtrx[0,0]);
    Ry=atan2(-mtrx[2,0],sqrt(mtrx[2,1]**2+mtrx[2,2]**2))
    Rx=atan2(mtrx[2,1],mtrx[2,2]);
    return [Rx*180/pi, Ry*180/pi, Rz*180/pi]

def RPY2mtrx(Rx, Ry, Rz):
    from math import pi,sin,cos
    Rx = Rx*pi/180
    Ry = Ry*pi/180
    Rz = Rz*pi/180
    return np.array([[cos(Rz)*cos(Ry), cos(Rz)*sin(Ry)*sin(Rx) - sin(Rz)*cos(Rx), sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx)],
    [sin(Rz)*cos(Ry), cos(Rz)*cos(Rx) + sin(Rz)*sin(Ry)*sin(Rx), sin(Rz)*sin(Ry)*cos(Rx)-cos(Rz)*sin(Rx)],
    [-sin(Ry), cos(Ry)*sin(Rx), cos(Ry)*cos(Rx)]])

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
    ,color = 'r', length=200, normalize=True),variable.quiver(xvecy, yvecy, zvecy, uvecy, vvecy, wvecy,
     length=200,color = 'g', normalize=True),variable.quiver(xvecz, yvecz, zvecz, uvecz, vvecz, wvecz,
     length=200, normalize=True)

fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

vis = vision()
length = 500
# camera coordinate in tool coordinate 
T1 = vis.eye2hand
# rotate tool coordinate coincident to camera coordinate 
[R,P,Y] = mtrx2RPY(T1)
print([R,P,Y])
R2 = RPY2mtrx(R,P,Y)
t2 =np.array([[300],[300],[300]])

ax.scatter(0, 0, 0, c='r' , marker = 'o')

coordinate(ax,T1[0:3][2:],T1[0:2][0:2])
# coordinate(ax,t2,R2)
coordinate(ax,[[0],[0],[0]],[[1,0,0],[0,1,0],[0,0,1]])

ax.set_xlabel('x')  
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim3d(-500,500)
ax.set_ylim3d(-500,500)
ax.set_zlim3d(-500,500)

plt.show()