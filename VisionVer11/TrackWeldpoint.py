import cv2 as cv
import numpy as np
from pypylon import pylon
from MyLibrary import vision, yaskawa
import time

vis = vision()
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.Open()
camera.Width.SetValue(1928)
camera.Height.SetValue(1208)
camera.StartGrabbing(pylon.GrabStrategy_LatestImages) 

grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
InitImg = grabResult.Array
pre = vis.Preprocessing(InitImg)
center = vis.LaserCenter(pre)
_, _, imgpos = vis.CD(center)
cv.circle(InitImg, (int(imgpos[0]),int(imgpos[1])), 10, 255, 10)
w = 600
h = 200
initBB = (int(imgpos[0] - w/2), int(imgpos[1] - h/2), w,h)
cv.rectangle(InitImg,(int(imgpos[0] - w/2), int(imgpos[1] - h/2)), (int(imgpos[0] + w/2), int(imgpos[1] + h/2)) ,255, 2)
tracker = cv.legacy.TrackerCSRT_create()
tracker.init(InitImg, initBB)
InitImg = cv.resize(InitImg, (1080, 720), interpolation = cv.INTER_AREA)
cv.imshow("", InitImg)
cv.waitKey(0)   

while camera.IsGrabbing():
    t = time.time()
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grabResult.GrabSucceeded():
        img = grabResult.Array
        (success, box) = tracker.update(img)
        # check to see if the tracking was a success
        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv.rectangle(img, (x, y), (x + w, y + h),255, 2)
            cv.circle(img, (int(x + w / 2), int(y + h /2)), 5, 255, 5)
        img = cv.resize(pre, (1080, 720), interpolation = cv.INTER_AREA)
        cv.imshow("", img)
        cv.waitKey(0)
        print("FPS: ",1/(time.time() - t))
        print("processing time: ", time.time() - t)