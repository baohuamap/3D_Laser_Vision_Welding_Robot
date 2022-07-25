'''
Laser Triangulation Technique
'''

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from MyLib import vision

vis = vision()
objp = vis.CalibParams()
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
intrinsic = vis.intrinsic
dist_coffs = vis.dist_coffs 
fx = intrinsic[0][0]
fy = intrinsic[1][1]
cx = intrinsic[0][2]
cy = intrinsic[1][2]
pointinlaserplane =[]

def ChessPlane(variable, position, rotation, planeNo):
    x = position[0][0]
    y = position[1][0]
    z = position[2][0]
    a = rotation[0][2]
    b = rotation[1][2]
    c = rotation[2][2]
    d = a*x + b*y + c*z

    x = np.linspace(-200,200,10)
    y = np.linspace(-200,200,10)
    X,Y = np.meshgrid(x,y)					
    Z= - a/c*X - b/c*Y + d/c
    return variable.plot_surface(X, Y, Z, color = planeNo, alpha=0.3)

def coordinate(variable, position, rotation, coordinateNo):
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

    return variable.quiver(xvecx, yvecx, zvecx, uvecx, vvecx, wvecx, length=50, color='r', normalize=True),\
            variable.quiver(xvecy, yvecy, zvecy, uvecy, vvecy, wvecy, length=50, color='g', normalize=True),\
            variable.quiver(xvecz, yvecz, zvecz, uvecz, vvecz, wvecz, length=50, color='b', normalize=True),\
            variable.text(position[0][0] , position[1][0], position[2][0], coordinateNo, fontsize=10)

def LaserPosition(chessPath,LaserPath):
    chessraw = cv.imread(chessPath)
    chess = cv.cvtColor(chessraw,cv.COLOR_BGR2GRAY)   
    laserraw = cv.imread(LaserPath)
    laser = cv.cvtColor(laserraw,cv.COLOR_BGR2GRAY)   
    rows,cols = laser.shape
    print("image shape:" ,rows,cols)

    ret, corners = cv.findChessboardCorners(chess, (10,7),None)
    corners_pos1 = cv.cornerSubPix(chess,corners,(11,11),(-1,-1), criteria)
    del corners
    chessraw = cv.drawChessboardCorners(chessraw, (10,7), corners_pos1,ret)

    if ret:
        retval, rvec_pos1, tvec1 = cv.solvePnP(objp,corners_pos1,intrinsic,dist_coffs)
        rotation_matrix1 = np.zeros(shape=(3,3))
        cv.Rodrigues(rvec_pos1, rotation_matrix1)

    laser = vis.Preprocessing(laser)
    temp1 = laser

    newcameramtx, _ =cv.getOptimalNewCameraMatrix(intrinsic,dist_coffs,(rows,cols),1,(rows,cols))
    temp2 = cv.undistort(chess, intrinsic, dist_coffs, None, newcameramtx)

    # Adaptive center extraction
    thinned = cv.ximgproc.thinning(laser)
    line = vis.LaserCenter(thinned)
    inv = np.linalg.inv(rotation_matrix1)
    for i in range(rows):
        for j in range(300,cols-300,1):
            if line[i][j] == 255:
                cv.circle(laserraw, (j,i), 5, [0,255,0], 2)
                Zc = (tvec1[0][0] * inv[2][0] +  tvec1[1][0] * inv[2][1] + tvec1[2][0] * inv[2][2])/(inv[2][0]/fx*(j-cx) + inv[2][1]/fy*(i-cy) + inv[2][2])
                C = np.array([Zc/fx*(j-cx), Zc/fy*(i-cy), Zc]).T.reshape(3,1)
                pointinlaserplane.append(C)
                
    laserraw[:,300] = 255
    laserraw[:,cols - 300] = 255
    laserraw = cv.resize(laserraw, (870,687), interpolation = cv.INTER_AREA)
    chessraw = cv.resize(chessraw, (870,687), interpolation = cv.INTER_AREA)
    temp1 = cv.resize(temp1, (870,687), interpolation = cv.INTER_AREA)
    temp2 = cv.resize(temp2, (870,687), interpolation = cv.INTER_AREA)
    result1 = np.concatenate((chessraw, laserraw), axis=1)
    result2 = np.concatenate((temp2, temp1), axis=1)
    del temp1,temp2
    cv.imshow("draw chess, draw laser on chess",result1)
    cv.imshow("undistorted chess, laser preprocessing",result2)
    cv.waitKey(0)
    cv.destroyAllWindows()

LaserPosition("D:\Code\Python_Code\WeldingRobot\VisionVer11\Mycamera/LaserCalibration/chess8.png",
              "D:\Code\Python_Code\WeldingRobot\VisionVer11\Mycamera/LaserCalibration/laser8.png")

# Least Square Error Plane Fitting__________________________________________________
x_square = 0
xy = 0  
x = 0
y_square = 0
y = 0
N = 0
xz = 0
yz = 0
z = 0

for i in range(len(pointinlaserplane)):
    x_square = x_square + pointinlaserplane[i][0][0]**2
    xy = xy + pointinlaserplane[i][0][0] * pointinlaserplane[i][1][0]
    x = x + pointinlaserplane[i][0][0]
    y_square = y_square + pointinlaserplane[i][1][0]**2
    y = y + pointinlaserplane[i][1][0]
    N = N + 1
    xz = xz + pointinlaserplane[i][0][0]*pointinlaserplane[i][2][0]
    yz = yz + pointinlaserplane[i][1][0]*pointinlaserplane[i][2][0]
    z = z + pointinlaserplane[i][2][0]

M = np.array([[x_square,  xy, x], [xy, y_square, y],[x, y, N]])
plane = np.linalg.inv(M).dot(np.array([[xz], [yz], [z]]))
print(plane)
del x_square
del xy
del x
del y_square
del y
del N
del xz
del yz
del z

# plot point in laser plane________________________________________
# X = np.array([0])
# Y = np.array([0])
# Z = np.array([0])

# for i in range(len(pointinlaserplane)):
#     X = np.insert(X, i, pointinlaserplane[i][0][0])
#     Y = np.insert(Y, i, pointinlaserplane[i][1][0])
#     Z = np.insert(Z, i, pointinlaserplane[i][2][0])

# fig = plt.figure()
# ax = fig.add_subplot(111, projection="3d")
# ax.scatter(X, Y, Z, c='r' , marker = '.')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# del X
# del Y
# del Z

# # plot laser plane_________________________________________________
# x = np.linspace(-100,100,10)
# y = np.linspace(-100,100,10)
# X,Y = np.meshgrid(x,y)
# Z=plane[0][0]*X + plane[1][0]*Y + plane[2][0]
# surf = ax.plot_surface( X, Y, Z, rstride=5, cstride=5, color='r', alpha=0.3)
# x = np.array([0,0,0])
# y = np.array([0,0,0])
# z = np.array([0,0,0])
# u = np.array([1,0,0])  
# v = np.array([0,1,0])
# w = np.array([0,0,1])

# # ax.quiver(x, y, z, u, v, w, length=50, normalize=True)
# # ChessPlane(ax,tvec1,rotation_matrix1, 'c')
# # coordinate(ax,tvec1,rotation_matrix1, 'O1')
# # ChessPlane(ax,tvec2,rotation_matrix2, 'm')
# # coordinate(ax,tvec2,rotation_matrix2, 'O2')
# # ChessPlane(ax,tvec3,rotation_matrix3, 'y')
# # coordinate(ax,tvec3,rotation_matrix3, 'O3')
# # ChessPlane(ax,tvec4,rotation_matrix4, 'g')
# # coordinate(ax,tvec4,rotation_matrix4, 'O3')
# # ChessPlane(ax,tvec5,rotation_matrix5, 'b')
# # coordinate(ax,tvec5,rotation_matrix5, 'O3')

# ax.text(50 , 0, 0 ,'X', fontsize=10)
# ax.text(0 , 50, 0 ,'Y', fontsize=10)
# ax.text(0 , 0, 50,'Z', fontsize=10)
# ax.set_xlim3d(-300,300)
# ax.set_ylim3d(-300,300)
# ax.set_zlim3d(1500,2100)
# plt.show()