import cv2 as cv
import numpy as np
import glob
from MyLib import Vision, savematrix
from skimage.measure import ransac, LineModelND
from GlobalVariables import *
from matplotlib import pyplot as plt
import tqdm

rows = 1200
cols = 1700

vis = Vision()
intrinsic = vis.intrinsic
dist_coffs = vis.dist_coffs
eye2hand = vis.eye2hand
[a, b, c, d] = vis.plane
fx = intrinsic[0][0]
fy = intrinsic[1][1]
cx = intrinsic[0][2]
cy = intrinsic[1][2]

# natural sorting https://stackoverflow.com/questions/5967500/how-to-correctly-sort-a-string-with-a-number-inside/5967539#5967539
import re

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]

def WeldSeamCenter(img):
    # adjust w to find the center of weldseam
    w = 5
    weldseam_center = []
    for j in range(1055, 1175, 1):
        d = []
        for i in range(350, 900, 1):
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
    for _ in range(5):
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

def calc_weldpoint2robot(point, pos):
    Zc                 = -d / (a/fx*(point[0]-cx) + b/fy*(point[1] - cy) + c)
    weldpoint2camera   = np.array([[Zc/fx*(point[0]-cx)], [Zc/fy*(point[1]-cy)], [Zc] , [1]])
    tool2robot         = vis.homogeneous(pos)
    weldpoint2robot    = tool2robot.dot(eye2hand).dot(weldpoint2camera).flatten()[0:3]
    return weldpoint2robot

WeldPoints = []
pos_no = 0

Images = glob.glob('D:/Workspace/Code/Welding_Robot/3D_Laser_Vision_Welding_Robot/MyCamera/Scan_data11/weldseam_' + '*.jpg')

Images.sort(key=natural_keys)

print(Images)
with open(scan_pos_path, 'r') as f:
    positions = [[float(num) for num in line.split('\t')] for line in f]

for image in Images:
    img = cv.imread(image, cv.IMREAD_GRAYSCALE)
    weldseam_center = WeldSeamCenter(img=img)
    data = np.array(weldseam_center)
    # RANSAC line fitting
    model = LineModelND()
    model_robust, inliers = ransac(data, LineModelND , min_samples=2,
                                residual_threshold=1, max_trials=1000)
    outliers = inliers == False
    line_x = [1060, 1170]
    line_y_robust = model_robust.predict_y(line_x)
    # weldseam line
    weldseam_point_1 = [line_x[0], line_y_robust[0]]
    weldseam_point_2 = [line_x[1], line_y_robust[1]]
    weldseam = (weldseam_point_1, weldseam_point_2)
    # laser center line
    # Laser center is the column 910 of the image
    laser_center_1 = [1115, 900]
    laser_center_2 = [1115, 1200]
    laser_centerline = (laser_center_1, laser_center_2)
    # find intersection
    feature_point = line_intersection(weldseam, laser_centerline)
    pos = positions[pos_no - 1]
    weldpoint2robot = calc_weldpoint2robot(feature_point, pos)
    weldpoint2robot[2] = -490
    print(weldpoint2robot)
    pos_no += 1
    WeldPoints.append(weldpoint2robot)
    # show img 
    for p in weldseam_center:
        cv.circle(img, (int(p[0]),int(p[1])), 1, (255, 255, 255), 1)
    img[:,1055] = 255
    img[:,1175] = 255

    frame = cv.cvtColor(img, cv.COLOR_GRAY2RGB)
    cv.circle(frame, (int(feature_point[0]),int(feature_point[1])), 5, (255, 0, 0), 5)
    frame = cv.resize(frame, (870,687), interpolation = cv.INTER_AREA)
    # cv.namedWindow("img", cv.WINDOW_AUTOSIZE)
    # cv.imshow("img", frame)
    # cv.waitKey()
    img_name = 'D:/Workspace/Code/Welding_Robot/3D_Laser_Vision_Welding_Robot/log/Scanning_processing_log/' + str(pos_no) + '.jpg'
    cv.imwrite(img_name, frame)    

WeldPoints = np.array(WeldPoints)
savematrix(welding_trajectory, WeldPoints)

# illustrate the solution
X = WeldPoints[:,0]
Y = WeldPoints[:,1]
Z = WeldPoints[:,2]

fig = plt.figure()
ax  = fig.add_subplot(111, projection="3d")
ax.scatter(X, Y, Z, c='r' , marker = '.')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
del X, Y, Z

plt.show()