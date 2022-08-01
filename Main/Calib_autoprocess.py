'''
Written by: Bao Hua
 
Code for automated calibration of the camera:
    1. Move the robot to the desired positions
    2. Grab the camera images
    3. Run the calibration code

'''


import os
import numpy as np
import cv2 as cv
from pypylon import pylon
from GlobalVariables import *
from math import pi,sin,cos
import threading
os.environ["PYLON_CAMEMU"] = "3"
import time
from MyLib import savematrix
import glob
import socket

# Params for NX100 controller **
DX100Address = "192.168.255.2"
DX100tcpPort = 80
CR = "\r"
CRLF = "\r\n"

# global variables
check_path = checkerboard_calib_cam_path
laser_path = checkerboard_calib_laser_path
robot_path = calib_trajectory           # Open when calibrating
R_path = R_path
t_path = t_path


class BaslerCam():
    def __init__(self,key):
        self.key = key
        self.image = 1
        self.started = True
        self._tlFactory = pylon.TlFactory.GetInstance()
        self._devices = self._tlFactory.EnumerateDevices()
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

    def grabImg(self):
        self.camera.Open()
        self.camera.StartGrabbing()
        printed = False

        while self.camera.IsGrabbing() and (self.started):
            if not self.camera.IsGrabbing():
                break
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            cameraContextValue = grabResult.GetCameraContext()
            if not printed:
                # print("Camera model",  ": ", self.camera[cameraContextValue].GetDeviceInfo().GetModelName())
                # Now, the image data can be processed.
                print("SizeX: ", grabResult.GetWidth())
                print("SizeY: ", grabResult.GetHeight())
                printed = True
            if grabResult.GrabSucceeded():
                self.image = grabResult.Array
            grabResult.Release()

    def stop(self):
        self.started = False
        cv.destroyAllWindows()
        self.camera.Close()

    def start(self):
        threading.Thread(target=self.grabImg, args=()).start()
        return self

class RobotTask():
    def __init__(self, key):
        self.key = key
        self.m = 0
        self.stopped = False
        self.CurrentPos = []
        self.model_count = 0
        self.robot_path = robot_path

    def utf8len(self, inputString):
        return len(inputString.encode('utf-8'))

    def command_data_length(self, command):
        if len(command) == 0:
            return 0
        else:
            return self.utf8len(command + CR)

    def read_pos_from_txt(self, pos):
        trajectory = open(self.robot_path, "r")
        for i, line in enumerate(trajectory):
            if i == pos - 1:
                command = line.rstrip()
        # self.CurrentPos = np.fromstring(command, dtype = float, sep=',')
        return command

    def robot_move_to_pos(self, command):
        #Comm setup
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        #Connect to the client/NX100 controller
        client.connect((DX100Address, DX100tcpPort))
        #START request
        startRequest = "CONNECT Robot_access" + CRLF
        client.send(startRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096)      #4096: buffer size
        startResponse = repr(response)
        print(startResponse)
        if 'OK: DX Information Server' not in startResponse:
            client.close()
            print('[E] Command start request response to DX100 is not successful!')
            return
        #COMMAND request
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "MOVJ" + " " + str(commandLength) + CRLF
        client.send(commandRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        print(commandResponse)
        if ('OK: ' + "MOVJ" not in commandResponse):
            client.close()
            print('[E] Command request response to NX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = command + (CR if len(command) > 0 else '')
            client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = client.recv(4096)
            commandDataResponse = repr(response)
            # print(commandDataResponse)
            if commandDataResponse:
                #Close socket
                client.close()
        time.sleep(0.01)

    def read_pos_from_robot(self):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        #Connect to the client/DX100 controller
        client.connect((DX100Address, DX100tcpPort))
        #START request
        startRequest = "CONNECT Robot_access" + CRLF
        client.send(startRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096)      #4096: buffer size
        startResponse = repr(response)
        print(startResponse)
        if 'OK: DX Information Server' not in startResponse:
            client.close()
            print('[E] Command start request response to DX100 is not successful!')
            return
        #COMMAND request
        command = "1, 0"
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "RPOSC" + " " + str(commandLength) + CRLF
        client.send(commandRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        print(commandResponse)
        if ('OK: ' + "RPOSC" not in commandResponse):
            client.close()
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            commandDataRequest = command + CR
            client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = client.recv(4096)
            commandDataResponse = repr(response)
            print(commandDataResponse)
            if commandDataResponse:
                client.close()
            time.sleep(0.01)
            commandDataResponse = commandDataResponse[2:50]
            self.CurrentPos = np.fromstring(commandDataResponse, dtype = float, sep=',')

    def RPY2mtrx(self):
        Rx = self.CurrentPos[3]*pi/180
        Ry = self.CurrentPos[4]*pi/180
        Rz = self.CurrentPos[5]*pi/180
        return np.array([[cos(Rz)*cos(Ry), cos(Rz)*sin(Ry)*sin(Rx) - sin(Rz)*cos(Rx), sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx)],
        [sin(Rz)*cos(Ry), cos(Rz)*cos(Rx) + sin(Rz)*sin(Ry)*sin(Rx), sin(Rz)*sin(Ry)*cos(Rx)-cos(Rz)*sin(Rx)],
        [-sin(Ry), cos(Ry)*sin(Rx), cos(Ry)*cos(Rx)]])

    def read(self):
        R = self.RPY2mtrx()
        t = np.array([self.CurrentPos[0], self.CurrentPos[1], self.CurrentPos[2]]).reshape(3,1)
        return R, t

    def save(self, path, mtx):
        cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
        cv_file.write("K1", mtx)
        cv_file.release()

    # def _save_model_poses(self, H, num):
    #     cv_file = cv.FileStorage('Model_poses' + str(num) + '.txt', cv.FILE_STORAGE_WRITE)
    #     cv_file.write("K1", H)
    #     cv_file.release()

    def _toHomo(self, R, t):
        c = np.array([[0, 0, 0, 1]])
        d = np.concatenate((R, t), axis = 1)
        d = np.concatenate((d, c))
        return d

    def process(self):
        # R_path
        # t_path
        R_end2base = []
        T_end2base = []
        H_end2base_model = []
        # pattern_count = 0
        pos_no = 1
        trajectory = open(self.robot_path, "r")
        waypoint = len(trajectory.readlines())
        print(f"There are {waypoint} waypoints in robot trajectory.")
        while not self.stopped:
            i = input()         # keyboard input
            if i == "r":
                if pos_no <= waypoint:
                    a = self.read_pos_from_txt(pos_no)
                    print(f"ROBOT TO WAYPOINT {pos_no}...")
                    self.robot_move_to_pos(a)
                    print(a)
                    time.sleep(2)
                    pos_no += 1
                    if pos_no == waypoint + 1:
                        print("----------ROBOT TRAJECTORY IS FINISHED----------\nInput 'r' to return...")
                else:
                    pos_no = 1
                    a = self.read_pos_from_txt(pos_no)
                    print("\nReturning to first waypoint...")
                    print(f"ROBOT TO WAYPOINT {pos_no}...")
                    self.robot_move_to_pos(a)
                    print(a)
                    time.sleep(2)
                    pos_no += 1
            elif i == 't':
                self.read_pos_from_robot()
                R, t = self.read()
                print(f'\nTRANSFORMATION AT WAYPOINT {pos_no-1}:')
                print('Rotation R:\n', R)
                print('Translation T:\n', t)
                R_end2base.append(R)
                T_end2base.append(t)
                time.sleep(0.5)
            elif i == 's':
                RR = np.array(R_end2base)
                TT = np.array(T_end2base)
                self.save(R_path, RR)
                self.save(t_path, TT)
                print('ALL WAYPOINT TRANSFORMATIONS SAVED!')
                time.sleep(3)

    def stop(self):
        self.stopped = True

    def start(self):
        threading.Thread(target=self.process, args=()).start()  


if __name__ == "__main__":
    choice = 1
    VisionSystem = BaslerCam(choice)
    VisionSystem.start()
    Robot = RobotTask(choice)
    Robot.start()

    # Window size reduction ratio
    t = 2
    check_count = 0
    prj_count = 0
    n = 0

    while True:
        img = VisionSystem.image
        frame = cv.resize(img,(720,480))
        cv.imshow('Frame',frame)
        choice = cv.waitKey(70)
        if choice & 0xFF == ord("c"):
            check_count = check_count + 1
            # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
            if check_count <10:
                filename = check_path +"\checkerboard_0" + str(check_count) +'.jpg'
            else:
                filename = check_path +"\checkerboard_" + str(check_count) +'.jpg'
            cv.imwrite(filename, img)
            print('Captured checkerboard image\nPair %d ' %(check_count))
        elif choice & 0xFF == ord("v"):
            # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
            if check_count <10:
                filename = laser_path +"\laser_0" + str(check_count) +'.jpg'
            else:
                filename = laser_path +"\laser_" + str(check_count) +'.jpg'
            cv.imwrite(filename, img)
            print('Captured laser image\nPair %d ' %(check_count))
        elif choice & 0xFF == ord("q"):
            VisionSystem.stop()
            Robot.stop()
            break
