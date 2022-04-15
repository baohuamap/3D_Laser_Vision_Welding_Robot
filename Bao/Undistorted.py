import cv2 as cv
from MyLibrary import vision

vis = vision()
mtx = vis.intrinsic
dist = vis.dist_coffs
img = cv.imread("D:/Desktop/Mycamera/CheckerBoard/1.png")
h,  w = img.shape[:2]
newcameramtx, roi=cv.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
dst = cv.resize(dst, (870,687), interpolation = cv.INTER_AREA)
cv.imshow('calibresult.png',dst)
cv.waitKey(0)