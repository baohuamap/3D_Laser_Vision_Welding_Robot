import cv2 as cv
import numpy as np
from pypylon import pylon
from MyLibrary import vision, yaskawa
import time
from MyLibrary import yaskawa

camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.Open()
camera.Width.SetValue(1928)
camera.Height.SetValue(1208)
cv.namedWindow("", cv.WINDOW_NORMAL)

# demonstrate some feature access
camera.StartGrabbing(pylon.GrabStrategy_LatestImages) 
tracker = cv.legacy.TrackerCSRT_create()
initBB = None
while camera.IsGrabbing():
   t = time.time()
   grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
   if grabResult.GrabSucceeded(): 
      img = grabResult.Array  
      if initBB is not None:
      # grab the new bounding box coordinates of the object
         (success, box) = tracker.update(img)
         # check to see if the tracking was a success
         if success: 
            (x, y, w, h) = [int(v) for v in box]
            cv.rectangle(img, (x, y), (x + w, y + h),0, 2)
            cv.circle(img, (int(x + w / 2), int(y + h /2)), 5, 255, 5)
            # print(int(x + w / 2), int(y + h /2))
            # print("Bounding box",w, h)
      cv.imshow("", img)
      key = cv.waitKey(10) & 0xFF 
      if key == ord("s"):
         initBB = cv.selectROI("", img, fromCenter=False,
            showCrosshair=True)
         tracker.init(img, initBB)
         print(initBB)
         cv.waitKey(0)
      elif key == ord("q"):
            break
      # print("FPS: ",1/(time.time() - t))
      # print("processing time: ", time.time() - t) 