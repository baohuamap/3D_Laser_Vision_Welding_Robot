import imghdr
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from skimage.measure import ransac, LineModelND

rows = 1200
cols = 1700

def WeldSeamCenter(img):
    # adjust w to find the center of weldseam
    w = 5
    weldseam_center = []
    for j in range(1084, 1184, 1):
    # for j in range(1165, 1225, 1):
        d = []
        for i in range(350, 800, 1):
            # if img[i][j] < 50:
            a = 2*(img[i-1,j]+img[i,j]+img[i+1,j])
            b = img[i+w,j]+img[i+w+1,j]+img[i+w+2,j]
            c = img[i-w,j]+img[i-w-1,j]+img[i-w-2,j]
            e = a - b - c
            d.append(e)
            del a
            del b
            del c
        min_val = min(d)
        center_point = [j, d.index(min_val)+350]
        weldseam_center.append(center_point)
        del d
    return weldseam_center

def LaserCenter(img):
        center = np.zeros((rows,cols))
        # find the center point
        for x in range(cols):
            sum1 = 0.0
            sum2 = 0.0
            roi = np.where(img[:,x] == 255)
            if roi[0].size != 0:
                for y in roi[0]:
                    sum1 += y * img[y][x]
                    sum2 += img[y][x]
                center[int(sum1/sum2)][x] = 255
        return center

def Preprocessing(img):
    blur = cv.GaussianBlur(img,(7,7),0)                 # use when img is gray image
    _, thresh = cv.threshold(blur,100,255,cv.THRESH_BINARY)
    closing = thresh
    del blur
    for _ in range(7):
        closing = cv.morphologyEx(closing, cv.MORPH_CLOSE, np.ones((7,7),np.uint8))
    del thresh
    return closing

# https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return int(x), int(y)

# print(line_intersection((A, B), (C, D)))

image = cv.imread("D:/Workspace/Code/Welding_Robot/3D_Laser_Vision_Welding_Robot/MyCamera/Scan_data12/weldseam_69.jpg", cv.IMREAD_GRAYSCALE)
# image = cv.imread("D:/Processing/Welding Robot Project/Photo/Test-RANSAC/img.jpg", cv.IMREAD_GRAYSCALE)
img = Preprocessing(image)
weldseam_center = WeldSeamCenter(img=image)
# thinned         = cv.ximgproc.thinning(img, thinningType=cv.ximgproc.THINNING_ZHANGSUEN)
# laser_center    = LaserCenter(img=thinned)
# print(laser_center)



data = np.array(weldseam_center)
# print(data)

# RANSAC line fitting
model = LineModelND ()
model_robust, inliers = ransac(data, LineModelND , min_samples=2,
                               residual_threshold=1, max_trials=1000)
outliers = inliers == False
line_x = [1036, 1156]
line_y_robust = model_robust.predict_y(line_x)

# weldseam line
weldseam_point_1 = [line_x[0], line_y_robust[0]]
weldseam_point_2 = [line_x[1], line_y_robust[1]]

weldseam = (weldseam_point_1, weldseam_point_2)

# laser center line
# Laser center is the column ... of the image
laser_center_1 = [1134, 900]
laser_center_2 = [1134, 1200]

laser_centerline = (laser_center_1, laser_center_2)

# find intersection
feature_point = line_intersection(weldseam, laser_centerline)

# ROI illustration
image[:,1184] = 255
image[:,1084] = 255

frame = cv.cvtColor(image, cv.COLOR_GRAY2RGB)

for p in weldseam_center:
    cv.circle(frame, (int(p[0]),int(p[1])), 1, (0, 0, 255), 1)
# laser ROI
# image[:,1135] = 255
# image[:,1095] = 255
cv.circle(frame, (int(feature_point[0]),int(feature_point[1])), 5, (255, 0, 0), 5)
frame = cv.resize(frame, (870,687), interpolation = cv.INTER_AREA)


# Show the Result
cv.namedWindow("img", cv.WINDOW_AUTOSIZE)
cv.imshow("img", frame)
cv.waitKey()


# #Plot the result of RANSAC
# fig, ax = plt.subplots()
# ax.plot(data[inliers, 0], data[inliers, 1], '.b', alpha=0.6,
#         label='Inlier data')
# ax.plot(data[outliers, 0], data[outliers, 1], '.r', alpha=0.6,
#         label='Outlier data')
# # ax.plot(line_x, line_y, '-k', label='Line model from all data')
# ax.plot(line_x, line_y_robust, '-b', label='Robust line model')
# ax.legend(loc='lower left')
# plt.show()
