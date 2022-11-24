import cv2 as cv
from pypylon import pylon
from MyLib import Yaskawa
import numpy as np
import threading
import time, sys
from GlobalVariables import *

Welding_HomePos = [1000, -16, -470, 180, 0, 0]

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

    def WeldProcess(self):
        '''
        1. move robot to home position
        2. move robot to weld
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
                pos[0] += 25
                pos[1] -= 15
                self.robot.MovL(self.pos2Movlcommand(pos))
                time.sleep(2)
                print('pos_no: ',pos_no)
                pos_no += 1
            elif pos_no <= waypoint:
                pos = positions[pos_no-1]
                pos[0] += 25
                pos[1] -= 15
                self.robot.MovL(self.pos2Movlcommand(pos))
                time.sleep(0.02)
                print('pos_no: ',pos_no)
                pos_no += 1
                if pos_no == waypoint + 1:
                    print('finished')
            else:
                StartWelding = False
        print('Welding - Done')        



    def run(self):
        global StartScanning, StartWelding, WeldPoints
        if StartWelding:
            self.WeldProcess()


StartWelding = True

robot    = RobotTask()

print("--- Homing process ---")
robot.homing()
time.sleep(3)
print("--- Homing process done ---")
print("--- Start welding ---")
 
# Start new Threads
robot.start()
