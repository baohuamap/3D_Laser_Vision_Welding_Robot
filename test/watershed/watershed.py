import cv2
import numpy as np

img = cv2.imread('D:\Code\Python_Code\WeldingRobot\Mycamera\Laser/11.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow('image', gray)

ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

# noise removal
kernel = np.ones((3,3), np.uint8)
opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations = 1)
# cv2.imshow('Open', opening)

# sure background area
sure_bg = cv2.dilate(opening, kernel, iterations = 1)
cv2.imshow('dilated', sure_bg)

# Finding sure foreground area
dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
ret, sure_fg = cv2.threshold(dist_transform,0.85*dist_transform.max(),255,0)

# Finding unknown region
sure_fg = np.uint8(sure_fg)
unknown = np.subtract(sure_bg, sure_fg)

cv2.imshow('unknown', unknown)

# Marker labelling
ret, markers = cv2.connectedComponents(sure_fg)
# print(markers)

## Add one so that sure background is not 0, but 1
markers = markers + 1

## Making the unknown area as 0
markers[unknown == 255] = 0
#cv2.imshow('markers2', markers)

markers = cv2.watershed(img, markers)
## boundary region is marked with -1
img[markers == -1] = (255, 0, 0)
cv2.imshow('watershed',img)
cv2.waitKey(0)