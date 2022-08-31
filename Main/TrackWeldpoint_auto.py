import cv2 as cv
from pypylon import pylon
from MyLib import Vision
import time

vis = Vision()
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.Open()
camera.Width.SetValue(1700)
camera.Height.SetValue(1200)
camera.OffsetX.SetValue(200)
camera.OffsetY.SetValue(8)
camera.StartGrabbing(pylon.GrabStrategy_LatestImages) 

tracker = cv.legacy.TrackerCSRT_create()
initBB = None

grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
InitImg = grabResult.Array
pre = vis.Preprocessing(InitImg)
thin = cv.ximgproc.thinning(pre, thinningType=cv.ximgproc.THINNING_ZHANGSUEN)
center = vis.LaserCenter(thin) 
cv.imshow("center", thin)
cv.waitKey(0)  
_, _, imgpos = vis.CD(center)
print(imgpos)

w = 400
h = 80
cv.circle(InitImg, (int(imgpos[0]), int(imgpos[1])), 5, 255, 5)
initBB = (int(imgpos[0] - w/2), int(imgpos[1] - h/2), w,h)
cv.rectangle(InitImg,(int(imgpos[0] - w/2), int(imgpos[1] - h/2)), (int(imgpos[0] + w/2), int(imgpos[1] + h/2)) ,255, 2)
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
        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv.rectangle(img, (x, y), (x + w, y + h),255, 2)
            cv.circle(img, (int(x + w / 2), int(y + h /2)), 5, 255, 5)
        else:
            print('false')
        img = cv.resize(img, (1080, 720), interpolation = cv.INTER_AREA)
        cv.imshow("", img)
        key = cv.waitKey(20) & 0xFF
        print("FPS: ",1/(time.time() - t))
        print("processing time: ", time.time() - t)