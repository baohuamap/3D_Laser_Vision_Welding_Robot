import cv2 as cv
import numpy as np
from pypylon import pylon
from MyLibrary import vision, yaskawa
import time
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.Open()

# demonstrate some feature access
camera.StartGrabbing(pylon.GrabStrategy_UpcomingImage) 
tracker = cv.legacy.TrackerCSRT_create()
initBB = None

while camera.IsGrabbing():
   t = time.time()
   grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
   if grabResult.GrabSucceeded():
      img = grabResult.Array
      img = cv.resize(img, (1080, 720), interpolation = cv.INTER_AREA)
      if initBB is not None:
      # grab the new bounding box coordinates of the object
         (success, box) = tracker.update(img)
         # check to see if the tracking was a success
         if success: 
            (x, y, w, h) = [int(v) for v in box]
            cv.rectangle(img, (x, y), (x + w, y + h),255, 2)
            cv.circle(img, (int(x + w / 2), int(y + h /2)), 5, 255, 5)
      cv.imshow("", img)
      key = cv.waitKey(10) & 0xFF 
      if key == ord("s"):
         initBB = cv.selectROI("", img, fromCenter=False,
            showCrosshair=True)
         tracker.init(img, initBB)
      elif key == ord("q"):
            break
      print("FPS: ",1/(time.time() - t))
      print("processing time: ", time.time() - t)