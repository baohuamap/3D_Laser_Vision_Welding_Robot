import cv2 as cv
from pypylon import pylon
import time
from MyLibrary import vision
import numpy as np
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.Open()

# demonstrate some feature access
camera.Width.SetValue(1928)
camera.Height.SetValue(1208)
camera.StartGrabbing(pylon.GrabStrategy_LatestImages) 
vis = vision()
out = cv.VideoWriter('basler.avi', 
    cv.VideoWriter_fourcc(*'I420'), 12, (1928,1208))
while camera.IsGrabbing():
    t = time.time()
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)  
    if grabResult.GrabSucceeded():
        img = grabResult.Array
        backtorgb = cv.cvtColor(img,cv.COLOR_GRAY2RGB)
        # pre = vis.Preprocessing(img)
        # thinned = cv.ximgproc.thinning(pre)
        # center = vis.TripleLaser(thinned)
        # _, _, imgpos = vis.CD(center)
        # imgpos[1] = imgpos[1]
        # cv.circle(img, (int(imgpos[0]),int(imgpos[1])), 10, 255, 10)
        img = cv.resize(img, (1080, 720), interpolation = cv.INTER_AREA)
        cv.imshow("", img)
        out.write(backtorgb)
        cv.waitKey(1)
        # print("grabbing and pr ocessing time: " , time.time() - t)
        print("fps: ", 1/(time.time() - t))
    else:
        print("grab fail")




# import cv2
# # function video capture
# cameraCapture = cv2.VideoCapture(0)
# # rame rate or frames per second
# fps = 30
 
# # Width and height of the frames in the video stream
# size = (int(cameraCapture.get(cv2.CAP_PROP_FRAME_WIDTH)),
# 			int(cameraCapture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
 
# """
# Create a VideoWriter object. We should specify the output file name (eg: MyOutput.avi). Then we should specify the FourCC. Then number of frames per second (fps) and frame size should be passed. May specify isColor flag. If it is True, encoder expect color frame, otherwise it works with grayscale frame.
# FourCC is a 4-byte code used to specify the video codec. The list of available codes can be found in fourcc.org. It is platform dependent.
# """
# out = cv2.VideoWriter('basler.avi', 
#     cv2.VideoWriter_fourcc('I','4','2','0'), 10, (640,480))
# print(size)
# success, frame = cameraCapture.read()
# frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)   
# # some variable
# numFramesRemaining = 10*fps - 1
 
# # loop until there are no more frames and variable > 0
# while success and numFramesRemaining > 0:
#     out.write(frame)
#     success, frame = cameraCapture.read()
#     cv2.imshow('frame',frame)
#     # print(frame.shape)
#     cv2.waitKey(1)
#     numFramesRemaining -= 1
# #Closes video file or capturing device
# cameraCapture.release()