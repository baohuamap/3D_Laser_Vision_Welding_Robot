import cv2 as cv
import numpy as np
from MyLibrary import vision

vis = vision()

src = cv.imread("D:\Code\Python_Code\WeldingRobot\VisionVer11\Mycamera\Position/2.png")
gray = cv.cvtColor(src,cv.COLOR_BGR2GRAY)   
rows,cols = gray.shape
img = vis.Preprocessing(gray)
thinned = cv.ximgproc.thinning(img)
temp = np.uint8(thinned)

lines= cv.HoughLines(temp, 1, np.pi/180.0, 100)

a1 = -np.cos(lines[0][0][1])/np.sin(lines[0][0][1])
b1 = lines[0][0][0]/np.sin(lines[0][0][1])
print("line1: ",a1,b1)
cv.line(temp,(1000,int(a1*1000+b1)),(3000,int(a1*3000+b1)),255,2)
a2 = -np.cos(lines[2][0][1])/np.sin(lines[2][0][1])
b2 = lines[2][0][0]/np.sin(lines[2][0][1])
print("line2: ",a2,b2)
cv.line(temp,(1000,int(a2*1000+b2)),(3000,int(a2*3000+b2)),255,2)
print(lines)

x = (b2-b1)/(a1-a2)
y = a1*x + b1
print("intersection" , x, y)
cv.circle(temp, (int(x),int(y)), 5, [255,0,0], 5)
cv.circle(src, (int(x),int(y)), 20, [255,0,0], 10)

# cv.circle(temp, (feature4pos[0], feature4pos[1]), 10, 255, 1) 
temp = cv.resize(temp, (1080, 720), interpolation = cv.INTER_AREA)
src = cv.resize(src, (1080, 720), interpolation = cv.INTER_AREA)

cv.imshow("result1",temp)
cv.imshow("result2",src)

cv.waitKey(0) 