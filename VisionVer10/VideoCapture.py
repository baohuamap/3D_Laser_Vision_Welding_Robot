# sourse: https://stackoverflow.com/questions/39272510/camera-calibration-with-circular-pattern/39328596
import cv2 as cv 
import glob
from MyLibrary import vision

vis = vision()
objp = vis.CalibParams()
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

out = cv.VideoWriter('outpy.mp4',cv.VideoWriter_fourcc(*'MP4V'), 10, (1732,1200), 0)

# Arrays to store object points and image points from all the ima ges.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
count = 0
images = glob.glob('D:/Desktop/Mycamera/CheckerBoard/*.png')    
for fname in images:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
    img = cv.imread(fname)
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)   
    ret, corners = cv.findChessboardCorners(gray, (10,7),None)
    if ret == True:
        objpoints.append(objp.astype('float32'))
        corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2) 
        # Draw and display the corners
        img = cv.drawChessboardCorners(img, (10,7), corners2,ret)
        cv.circle(img, (int(corners2[0][0][0]),int(corners2[0][0][1])), 20, [255,0,0], 10)
        resized = cv.resize(img, (960, 520), interpolation = cv.INTER_AREA)
        out.write(gray)
        print(gray.shape)
        cv.imshow('img',resized)
        count = count + 1
        cv.waitKey(100)
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print("RMS: " ,ret)
print("total of images:" ,count)
print("intrinsic", mtx)
print("dist_coffs", dist)
cv.destroyAllWindows()
        