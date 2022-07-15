# import the necessary packages
# import numpy as np
import time
import cv2 as cv
from MyLibrary import vision

# tracker = cv.legacy.TrackerMOSSE_create()
# tracker = cv.legacy.TrackerKCF_create()
tracker = cv.legacy.TrackerCSRT_create()
initBB = None
track_pts = []

video_dir = 'D:/Code/Welding_Robot/DataSet/Video'

vis = vision()

vid = cv.VideoCapture(f'{video_dir}/17V0A.avi')
InitImg = vid.read()[1]
pre = vis.Preprocessing(InitImg)
cv.imshow('pre', pre)
cv.waitKey(0)
center = vis.TripleLaser(pre)
cv.imshow('pre', center)
cv.waitKey(0)
_, _, imgpos = vis.CD(center)
print(imgpos)
cv.circle(InitImg, (int(imgpos[0]),int(imgpos[1])), 10, 255, 10)

w, h = 60, 60

initBB = (int(imgpos[0] - w/2), int(imgpos[1] - h/2), w,h)
cv.rectangle(InitImg,(int(imgpos[0] - w/2), int(imgpos[1] - h/2)), (int(imgpos[0] + w/2), int(imgpos[1] + h/2)) ,255, 2)
tracker = cv.legacy.TrackerCSRT_create()
tracker.init(InitImg, initBB)
InitImg = cv.resize(InitImg, (1080, 720), interpolation = cv.INTER_AREA)
cv.imshow("", InitImg)
cv.waitKey(0)

frame = 0
fps = None

while frame is not None:
    t = time.time()
    _, frame = vid.read()
    frame = cv.resize(frame, (1080,720))
    pre = vis.Preprocessing(InitImg)
    cv.imshow('pre', pre)
    cv.waitKey(0)
    center = vis.TripleLaser(pre)
    cv.imshow('pre', center)
    cv.waitKey(0)
    _, _, imgpos = vis.CD(center)
    print(imgpos)
    cv.circle(InitImg, (int(imgpos[0]),int(imgpos[1])), 10, 255, 10)

    if initBB is not None:
        (success, box) = tracker.update(frame)
        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv.rectangle(frame, (x, y), (x + w, y + h),255, 2)
            cv.circle(frame, (int(x + w / 2), int(y + h /2)), 5, 255, 5)
    cv.imshow("Frame", frame)
    cv.waitKey(0)
    print("FPS: ",1/(time.time() - t))
    print("processing time: ", time.time() - t)

#     key = cv.waitKey(50) & 0xFF 
#     if key == ord("s"):
#             initBB = cv.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
#             cv.waitKey(0)
#             tracker.init(frame, initBB)
#             print(initBB)
#     if key == ord("a"):
#             cv.waitKey(100)
#             print(track_pts)

            # track_point = [(int(x + w / 2), int(y + h /2))]
            # cv.rectangle(frame, (x, y), (x + w, y + h), 0, 2)
            # cv.circle(frame, track_point[0], track_point[1], 5, 255, 5)
            # track_pts = []