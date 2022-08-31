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

while camera.IsGrabbing():
    t = time.time()
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grabResult.GrabSucceeded():
        img = grabResult.Array
        img = cv.resize(img, (1080, 720), interpolation = cv.INTER_AREA)
        if initBB is not None:
            (success, box) = tracker.update(img)
            if success:
                (x, y, w, h) = [int(v) for v in box]
                cv.rectangle(img, (x, y), (x + w, y + h),255, 2)
                cv.circle(img, (int(x + w / 2), int(y + h /2)), 5, 255, 5)
        img = cv.resize(img, (1080, 720), interpolation = cv.INTER_AREA)
        cv.imshow("", img)
        key = cv.waitKey(20) & 0xFF 
        if key == ord("s"):
            initBB = cv.selectROI("", img, fromCenter=False,
                showCrosshair=True)
            tracker.init(img, initBB)
        elif key == ord("q"):
                break
        print("FPS: ",1/(time.time() - t))
        print("processing time: ", time.time() - t)