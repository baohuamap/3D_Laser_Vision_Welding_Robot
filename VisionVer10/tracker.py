# import the necessary packages
import numpy as np
import time
import cv2 as cv

# tracker = cv.legacy.TrackerMOSSE_create()
# tracker = cv.legacy.TrackerKCF_create()
tracker = cv.legacy.TrackerCSRT_create()
initBB = None
track_pts = []

vs = cv.VideoCapture('D:\Code\Python_Code\WeldingRobot\Bao\DataSet/19.6V100A2.avi')
frame = 0
fps = None 	
while frame is not None:
    _, frame = vs.read()
    frame = cv.resize(frame, (1080,720))
    if initBB is not None:
        t = time.time()
        (success, box) = tracker.update(frame)
        if success:
            [x, y, w, h] = [int(v) for v in box]
            track_point = [int(x + w / 2), int(y + h /2)]
            cv.rectangle(frame, (x, y), (x + w, y + h), 0, 2)
            cv.circle(frame, (track_point[0], track_point[1]), 5, 255, 5)
            track_pts.append(track_point)
            # print("track success")
            print("tracking time: ", time.time() - t)
    cv.imshow("Frame", frame)
    key = cv.waitKey(50) & 0xFF 
    if key == ord("s"):
            initBB = cv.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
            cv.waitKey(0)
            tracker.init(frame, initBB)
            print(initBB)
    if key == ord("a"):
            cv.waitKey(100)
            print(track_pts)

            # track_point = [(int(x + w / 2), int(y + h /2))]
            # cv.rectangle(frame, (x, y), (x + w, y + h), 0, 2)
            # cv.circle(frame, track_point[0], track_point[1], 5, 255, 5)
            # track_pts = []