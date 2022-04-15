import cv2 as cv
from pypylon import pylon
import time
from MyLibrary import vision
import numpy as np
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.Open()

# demonstrate some feature access
# camera.Width.SetValue(3840)
# camera.Height.SetValue(2748)
camera.StartGrabbing(pylon.GrabStrategy_UpcomingImage) 
vis = vision()
while camera.IsGrabbing():
    t = time.time()
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grabResult.GrabSucceeded():
        img = grabResult.Array
        # img = vis.Preprocessing(img)
        # temp = np.uint8(vis.LaserCenter(img))
        # feature1,feature2 = vis.CD(temp)
        # cv.circle(temp, (feature1[0], feature1[1]), 10, 255, 5) 
        # cv.circle(temp, (feature2[0], feature2[1]), 10, 255, 5) 
        temp = cv.resize(img, (1080, 720), interpolation = cv.INTER_AREA)
        cv.imshow("", temp)
        cv.waitKey(1)
        print("grabbing and processing time: " , time.time() - t)
        print("fps: ", 1/(time.time() - t))
    else:
        print("grab fail")