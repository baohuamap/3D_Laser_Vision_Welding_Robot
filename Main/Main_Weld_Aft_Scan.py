from matplotlib import pyplot as plt
from pypylon import pylon
import cv2 as cv
from MyLib import Yaskawa
import numpy as np
import threading
import time, sys
from GlobalVariables import *

# --- Choose the Welding_HomePos ---
Welding_HomePos = [668, -30, 20, 180, -25, 0]

StartWelding = True

class RobotTask(threading.Thread):
    def __init__(self):
        global CurrentPos, StopProcess
        threading.Thread.__init__(self)
        StopProcess = False
        self.robot = Yaskawa()
        self.robot.StartRequest()
        CurrentPos = Welding_HomePos

    def pos2Movlcommand(self, pos):
        position = ""
        for i in pos:
            position += str(i) + ", "
        command = "0, 20, 0, " + position + "180, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
        return command

    def pos2Movjcommand(self, pos):
        position = ""
        for i in pos:
            position += str(i) + ", "
        command = "20, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
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

    def WeldProcess(self):
        '''
        1. move robot to home position
        2. move robot AND weld
        '''
        global StartWelding
        pos_no = 1
        
        with open(welding_trajectory_filtered, 'r') as f:
            positions = [[round(float(num),3) for num in line.split('\t')] for line in f]

        waypoint = len(positions)
        print('waypoint: ', waypoint)

        StartWelding = True 
        while StartWelding:
            if pos_no == 1:
                pos = positions[pos_no-1]
                pos[0] += 9
                pos[1] -= 15
                self.robot.MovL(self.pos2Movlcommand(pos))
                time.sleep(3)
                self.robot.Stop()
                self.robot.StartRequest()
                on = self.robot.ArcOn()
                print('pos_no: ',pos_no)
                print(on)
                pos_no += 1
            elif pos_no <= waypoint:
                pos = positions[pos_no-1]
                pos[0] += 9
                pos[1] -= 15
                self.robot.MovL(self.pos2Movlcommand(pos))
                time.sleep(0.02)
                print('pos_no: ',pos_no)
                pos_no += 1
                if pos_no == waypoint + 1:
                    print('finished')
                    endpos = positions[waypoint-1]
                    endpos[0] += 9
                    endpos[1] -= 15
                    endpos[2] += 50
                    self.robot.MovL(self.pos2Movlcommand(endpos))
                    time.sleep(1)
                    self.robot.Stop()
                    self.robot.StartRequest()
                    off = self.robot.ArcOff()
                    print(off)
                    print('end pos')
            else:
                StartWelding = False
        print('Welding - Done')        



    def run(self):
        global StartScanning, StartWelding, WeldPoints
        if StartWelding:
            self.WeldProcess()

class CameraTask(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        self.camera.Width.SetValue(1700)
        self.camera.Height.SetValue(1200)
        self.camera.OffsetX.SetValue(8)
        self.camera.OffsetY.SetValue(8)
        self.camera.ExposureTimeAbs = 20000 
        self.camera.StartGrabbing() 
        
    def run(self):
        global StopProcess
        while self.camera.IsGrabbing() and not StopProcess:
            try:
                grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    self.image = grabResult.Array
            except:
                print("Camera error: ",sys.exc_info())


if __name__ == "__main__":
    with open(welding_trajectory_filtered, 'r') as f:
        positions = [[round(float(num),3) for num in line.split('\t')] for line in f]

    positions = np.array(positions)
    X = positions[:,0]
    Y = positions[:,1]
    Z = positions[:,2]
    fig = plt.figure()
    ax  = fig.add_subplot(111, projection="3d")
    ax.scatter(X, Y, Z, c='r' , marker = '.')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

    robot = RobotTask()
    cam   = CameraTask()

    print("--- Homing process ---")
    robot.homing()
    time.sleep(5)
    print("--- Homing process done ---")
    print("--- Start welding ---")
    
    # Start new Threads
    robot.start()
    # cam.start()
    # while True:
    #     img = cam.image
    #     frame = cv.resize(img,(720,480))
    #     cv.imshow('Frame',frame)