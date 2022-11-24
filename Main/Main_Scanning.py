import cv2 as cv
from pypylon import pylon
from MyLib import Vision, Yaskawa, savematrix
import numpy as np
import threading
import time, sys
from GlobalVariables import *

Welding_HomePos = [736, -35, -464, 180, 7, -5] # butt welding spline

# Scan_StartPos = [918, -50, -450, -180, 0, 0]
# Scan_EndPos = [1190, -50, -450,	-180,	0, 0]
# Scan_StartPos = [792, -35, -300, 180, 7, -5]
# Scan_EndPos = [1061, -35, -300, 180, 7, -5]
Scan_StartPos = [737, -35, -464, 180, 7, -5]
Scan_MidPos = [815, -67, -464, 180, 7, -5]
Scan_EndPos = [1025, -9, -464, 180, 7, -5]
# Scan_EndPos = [1025, -35, -464, 180, 7, -5]


off = False
on  = True

class RobotTask(threading.Thread):
    def __init__(self):
        global CurrentPos, StopProcess
        threading.Thread.__init__(self)
        StopProcess = False
        self.robot = Yaskawa()
        self.robot.StartRequest()
        CurrentPos = Welding_HomePos

    def read_pos_from_txt(self, filename, pos_no):
        trajectory = open(filename, "r")
        for i, line in enumerate(trajectory):
            if i == pos_no - 1:
                command = line.rstrip()
        # self.CurrentPos = np.fromstring(command, dtype = float, sep=',')
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
        command = self.pos2Movjcommand(Welding_HomePos)
        self.robot.MovJ(command)

    def stop(self):
        global StopProcess, StartWelding
        self.robot.Stop()
        StopProcess = True
        StartWelding = False
        print("weld done")

    def TurnLED(self, status):
        if status == True:
            ...
        else:
            ...

    def SaveTrajectory(self):
        # Save trajectory to compare with 3d model trajectory
        pass

    def ScanProcess(self):
        '''
        1. move robot to home position
        2. move robot to scan the workpiece
        '''        
        global StartScanning
        # init_point = [913, -50.5, -460, -180, 0, 0] # Start position
        self.robot.MovJ(self.pos2Movjcommand(Scan_StartPos))
        StartScanning = True
        '''
        while StartScanning:
            if pos_no <= waypoint:
                pos = self.read_pos_from_txt(scan_trajectory_path, pos_no)
                self.robot.MovL(pos)
                time.sleep(1)
                ImgArr.append(CurrImg)
                CurrentPos = self.robot.RposC()
                NowPos.append(CurrentPos)
                pos_no += 1
                if pos_no == waypoint + 1:
                    print('finished')
            else:
                StartScanning = False
                self.TurnLED(False)
        print('Scanning - Done')
        '''
        while StartScanning:
            self.robot.MovL(self.pos2Movlcommand(Scan_MidPos))
            Scan2MidPos = True
            count = 1
            while Scan2MidPos:
                if count < 10:
                    img_name = scan_weld_seam_img_path + "\weldseam_0" + str(count) + '.jpg'
                else:
                    img_name = scan_weld_seam_img_path + "\weldseam_" + str(count) + '.jpg'
                CurrentPos = self.robot.RposC()
                NowPos.append(CurrentPos)
                cv.imwrite(img_name, CurrImg)
                print('pos no: ', count)
                # time.sleep(0.01)
                count += 1
                if CurrentPos[0] >= 814.5:
                    Scan2MidPos = False
            self.robot.MovL(self.pos2Movlcommand(Scan_EndPos))
            while True:
                if count < 10:
                    img_name = scan_weld_seam_img_path + "\weldseam_0" + str(count) + '.jpg'
                else:
                    img_name = scan_weld_seam_img_path + "\weldseam_" + str(count) + '.jpg'
                CurrentPos = self.robot.RposC()
                NowPos.append(CurrentPos)
                cv.imwrite(img_name, CurrImg)
                print('pos no: ', count)
                # time.sleep(0.01)
                count += 1
                if CurrentPos[0] >= 1024.5:
                    StartScanning = False
                    self.TurnLED(off)
                    savematrix(scan_pos_path, NowPos)
                    break
        print('scanning done')
        # print('ImgArr length:\n', len(ImgArr))
        print('NowPos length:\n', len(NowPos))



    def run(self):
        global StartScanning, StartWelding, WeldPoints
        if StartScanning and not StartWelding:
            self.TurnLED(on)
            self.ScanProcess()
            
        if StartWelding and not StartScanning:
 
            InitPos = WeldPoints[0]
            command = self.pos2Movlcommand(InitPos)
            self.robot.MovL(command)
            self.robot.ArcOn()
            for pos in WeldPoints:
                command = self.pos2Movlcommand(pos)
                self.robot.MovL(command)
            self.robot.ArcOff()
            pass

        '''
        Scan xong sẽ hàn 
        '''


class CameraTask(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        self.camera.Width.SetValue(1700)
        self.camera.Height.SetValue(1200)
        self.camera.OffsetX.SetValue(8)
        self.camera.OffsetY.SetValue(8)
        self.camera.ExposureTimeAbs = 30000 
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

camera   = CameraTask()
robot    = RobotTask()
software = SoftwareTask()

print("--- Homing process ---")
robot.homing()
time.sleep(5)
print("--- Homing process done ---")
print("--- Start welding ---")
 
# Start new Threads
camera.start()
robot.start()

# frame = cv.resize(CurrImg,(720,480))
# cv.imshow('Frame',frame)
# software.start()