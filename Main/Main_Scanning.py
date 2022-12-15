import cv2 as cv
from pypylon import pylon
from MyLib import Vision, Yaskawa, savematrix
import numpy as np
import threading
import time, sys
from GlobalVariables import *

# ------------------- Choose the Scan_HomePos --------------------------------
Scan_HomePos = [666, -30, 20, 180, -25, 0] # butt welding spline

Scan_StartPos = [668, -30, 20, 180, -25, 0]
Scan_EndPos = [939, -30, 20, 180, -25, 0]

class RobotTask(threading.Thread):
    def __init__(self):
        global StopProcess
        threading.Thread.__init__(self)
        StopProcess = False
        self.robot = Yaskawa()
        self.robot.StartRequest()
        
    def read_pos_from_txt(self, filename, pos_no):
        trajectory = open(filename, "r")
        for i, line in enumerate(trajectory):
            if i == pos_no - 1:
                command = line.rstrip()
        return command

    def pos2Movlcommand(self, pos):
        position = ""
        for i in pos:
            position += str(i) + ", "
        command = "0, 10, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
        return command

    def pos2Movjcommand(self, pos):
        position = ""
        for i in pos:
            position += str(i) + ", "
        command = "10, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
        return command

    def homing(self):
        command = self.pos2Movjcommand(Scan_HomePos)
        self.robot.MovJ(command)

    def stop(self):
        global StopProcess, StartWelding
        StopProcess = True
        StartWelding = False
        print("Scan done")
        self.robot.Stop()

    def ScanProcess(self):
        '''
        1. move robot to home position
        2. move robot to scan the workpiece
        '''        
        global StartScanning
        self.robot.MovJ(self.pos2Movjcommand(Scan_StartPos))
        StartScanning = True
        time.sleep(0.05)
        count = 1
        while StartScanning:
            self.robot.MovL(self.pos2Movlcommand(Scan_EndPos))
            time.sleep(0.05)
            while True:
                img_name = scan_weld_seam_img_path + "\weldseam_" + str(count) + '.jpg'
                CurrentPos = self.robot.RposC()
                NowPos.append(CurrentPos)
                cv.imwrite(img_name, CurrImg)
                print('pos no: ', count)
                time.sleep(0.05)
                count += 1
                if CurrentPos[0] >= 938.5:
                    StartScanning = False
                    savematrix(scan_pos_path, NowPos)
                    self.stop()
                    break
            print('scanning done')
        print('ImgArr length:\n', len(ImgArr))
        print('NowPos length:\n', len(NowPos))

    def run(self):
        global StartScanning, StartWelding, WeldPoints
        if StartScanning and not StartWelding:
            self.ScanProcess()

class CameraTask(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        self.camera.Width.SetValue(1700)
        self.camera.Height.SetValue(1200)
        self.camera.OffsetX.SetValue(8)
        self.camera.OffsetY.SetValue(8)
        self.camera.ExposureTimeAbs = 10000 
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImages) 
        
    def run(self):
        global CurrImg, StopProcess
        while self.camera.IsGrabbing() and not StopProcess:
            try:
                grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    CurrImg = grabResult.Array
            except:
                print("Camera error: ",sys.exc_info())
    
class SoftwareTask(threading.Thread):
   def __init__(self):
      threading.Thread.__init__(self)
      self.vis = Vision()
      self.intrinsic = self.vis.intrinsic
      self.dist_coffs = self.vis.dist_coffs
      self.eye2hand = self.vis.eye2hand
      [self.a, self.b, self.c, self.d] = self.vis.plane
      self.fx = self.intrinsic[0][0]
      self.fy = self.intrinsic[1][1]
      self.cx = self.intrinsic[0][2]
      self.cy = self.intrinsic[1][2]

   def run(self):
        pass

ImgArr = []
StartScanning = True
StartWelding = False
WeldDone = False
NowPos = []
WeldPoints = []
CurrentPos = []

camera   = CameraTask()
robot    = RobotTask()
software = SoftwareTask()

print("--- Homing process ---")
robot.homing()
time.sleep(3)
print("--- Homing process done ---")
print("--- Start Scanning ---")
 
# Start new Threads
camera.start()
robot.start()
