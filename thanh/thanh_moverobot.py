# from multiprocessing.dummy import current_process
import os
from pickle import TRUE
import numpy as np
import cv2 as cv
from pypylon import pylon
# from GlobalVariables import *
import math
from math import pi,sin,cos
import threading
os.environ["PYLON_CAMEMU"] = "3"
import time
# from save_load_lib import savematrix
import glob
import socket

# Params for NX100 controller
nx100Address = "169.254.3.45"
nx100tcpPort = 80
CR = "\r"
CRLF = "\r\n"

# check_left = checkerboard_path_left
# check_right = checkerboard_path_right

class BaslerCam():
    def __init__(self,key):
        self.key = key
        self.number = 10
        self.im_l = 1
        self.im_r = 1
        self.started = True
        maxCamerasToUse = 2
        self._tlFactory = pylon.TlFactory.GetInstance()
        self._devices = self._tlFactory.EnumerateDevices()
        self.cameras = pylon.InstantCameraArray(min(len(self._devices), maxCamerasToUse))
        print(min(len(self._devices), maxCamerasToUse))
        for i, cam in enumerate(self.cameras):
            cam.Attach(self._tlFactory.CreateDevice(self._devices[i]))
            # Print the model name of the camera.
            print("Using device ", cam.GetDeviceInfo().GetModelName())
    def grabImg(self):
        self.cameras.Open()
        self.cameras.StartGrabbing()
        printed = False
        while self.cameras.IsGrabbing() and (self.started):
            if not self.cameras.IsGrabbing():
                break
            grabResult1 = self.cameras[0].RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            grabResult2 = self.cameras[1].RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            cameraContextValue = grabResult1.GetCameraContext()
            if not printed:
                print("Camera model",  ": ", self.cameras[cameraContextValue].GetDeviceInfo().GetModelName())
                # Now, the image data can be processed.
                print("SizeX: ", grabResult1.GetWidth())
                print("SizeY: ", grabResult1.GetHeight())
                printed = True
            if grabResult1.GrabSucceeded() and grabResult2.GrabSucceeded():
                self.im_l = grabResult1.Array
                self.im_r = grabResult2.Array
            grabResult1.Release()
            grabResult2.Release()
    def stop(self):
        self.started = False
        cv.destroyAllWindows()
        self.cameras.Close()
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
        self.trajectory_path = "D:/Code/Python_Code/WeldingRobot/RobotControl/trajectory.txt"

    def utf8len(self,inputString):
        return len(inputString.encode('utf-8'))

    def command_data_length(self, command):
        if len(command) == 0:
            return 0
        else:
            return self.utf8len(command + CR)

    def read_pos_from_txt(self, pos):
        trajectory = open(self.trajectory_path, "r")
        for i, line in enumerate(trajectory):
            if i == pos - 1:
                command = line.rstrip()
        self.CurrentPos = np.fromstring(command, dtype=float, sep=',')
        return command

    def robot_move_to_pos(self, command):
        #Comm setup
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        #Connect to the client/NX100 controller
        client.connect((nx100Address, nx100tcpPort))
        #START request
        startRequest = "CONNECT Robot_access" + CRLF
        client.send(startRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096)      #4096: buffer size
        startResponse = repr(response)
        print(startResponse)
        if 'OK: NX Information Server' not in startResponse:
            client.close()
            print('[E] Command start request response to NX100 is not successful!')
            return
        #COMMAND request
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "MOVL" + " " + str(commandLength) + CRLF
        client.send(commandRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        print(commandResponse)
        if ('OK: ' + "MOVL" not in commandResponse):
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

    def RPY2mtrx(self):
        Rx = self.CurrentPos[6]*pi/180
        Ry = self.CurrentPos[7]*pi/180
        Rz = self.CurrentPos[8]*pi/180
        return np.array([[cos(Rz)*cos(Ry), cos(Rz)*sin(Ry)*sin(Rx) - sin(Rz)*cos(Rx), sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx)],
        [sin(Rz)*cos(Ry), cos(Rz)*cos(Rx) + sin(Rz)*sin(Ry)*sin(Rx), sin(Rz)*sin(Ry)*cos(Rx)-cos(Rz)*sin(Rx)],
        [-sin(Ry), cos(Ry)*sin(Rx), cos(Ry)*cos(Rx)]])

    def posc2Rt(self):
        R = self.RPY2mtrx()
        t = np.array([self.CurrentPos[3], self.CurrentPos[4] ,self.CurrentPos[5]]).reshape(3,1)
        return R, t
    
    def read(self):
        R, t = self.posc2Rt()
        return R, t

    def save(self, path, mtx):
        cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
        cv_file.write("K1", mtx)
        cv_file.release()

    def _save_model_poses(self, H, num):
        cv_file = cv.FileStorage('Model_poses'+str(num)+'.txt', cv.FILE_STORAGE_WRITE)
        cv_file.write("K1", H)
        cv_file.release()

    def _toHomo(self, R, t):
        c = np.array([[0,0,0,1]])
        d = np.concatenate((R,t),axis = 1)
        d = np.concatenate((d,c))
        return d

    def process(self):
        R_path = 'R.txt'
        t_path = 't.txt'
        R_end2base = []
        T_end2base = []
        H_end2base_model =[]
        pattern_count = 0 
        pos_no = 1
        trajectory = open(self.trajectory_path, "r")
        max_pos = len(trajectory.readlines())
        print(max_pos)
        while not self.stopped:
            i = input()         # keyboard input
            if i == "r":
                if pos_no <= max_pos:
                    a = self.read_pos_from_txt(pos_no)
                    # self.robot_move_to_pos(a)
                    print(a)
                    print(pos_no)
                    pos_no += 1
                else:
                    pos_no = 1
                    a = self.read_pos_from_txt(pos_no)
                    # self.robot_move_to_pos(a)
                    print(a)
                    print(pos_no)
                    pos_no += 1 
            elif i == 'c':
                R,t = self.read()
                print('\nCurrent Pos is\n')
                print('R:\n',R)
                print('T:\n',t)
                R_end2base.append(R)
                T_end2base.append(t)
                time.sleep(0.5)
            elif i == 's':
                RR= np.array(R_end2base)
                TT = np.array(T_end2base)
                self.save(R_path,RR)
                self.save(t_path,TT)
                print('Saved')
                time.sleep(3)
            elif i == 'p':
                # pattern_count= pattern_count+1
                # if pattern_count == 10:
                R,t = self.read()
                H = self._toHomo(R,t)
                H_end2base_model.append(H)
                H = np.array(H_end2base_model)
                self._save_model_poses(H,self.model_count)
                print('Record model position ',self.model_count)
                # pattern_count = 0
                time.sleep(1)
    def stop(self):
        self.stopped = True
    def start(self):
        threading.Thread(target=self.process, args=()).start()

if __name__ == "__main__":
    choice = 1
    # StereoSystem = BaslerCam(choice)
    Robot = RobotTask(choice)
    # StereoSystem.start()
    Robot.start()

    # Window size reduction ratio
    t = 2
    # prj_im_list = glob.glob(projecting_pattern +'*.jpg')    
    check_count = 0
    prj_count = 0
    n = 0

    # First cloud count
    model_count = 5

    # while True:
        
    #     # prj_im = cv.imread(prj_im_list[prj_count])
    #     # img_l = StereoSystem.im_l
    #     # img_r = StereoSystem.im_r
    #     # frame = np.hstack([cv.resize(img_l,(720,480)),cv.resize(img_r,(720,480))]) 
    #     # cv.imshow('Frame',frame)
    #     # cv.imshow('Pattern',cv.resize(prj_im,(int(1920/t),int(1080/t))))
    #     choice = cv.waitKey(70)

    #     Robot.key = choice
    #     Robot.model_count = model_count
    #     Robot.process()


        # elif choice & 0xFF == ord("c"):
        #     check_count = check_count + 1
        #     # print(count)
        #     # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
        #     if check_count <10:
        #         filename_l = check_left +"left0" + str(check_count) +'.bmp'
        #         filename_r = check_right +"right0"+ str(check_count) +'.bmp'
        #     else:
        #         filename_l = check_left +"left" + str(check_count) +'.bmp'
        #         filename_r = check_right +"right"+ str(check_count) +'.bmp'
        #     # cv.imwrite(filename_l, img_l)
        #     # cv.imwrite(filename_r, img_r)
        #     print('Captured checkerboard image\nPair %d ' %(check_count))
        # elif choice & 0xFF == ord("p"):
        #     n = n+1
        #     if n <10:
        #         filename_l = model_path +"model" + str(model_count) + '/Left/left0'+ str(n) + '.bmp'
        #         filename_r = model_path +"model" + str(model_count) + '/Right/right0'+ str(n) + '.bmp'
        #         print('Captured model %d image\nPair %d ' %(model_count, n))
        #     if n ==10:
        #         filename_l = model_path +"model" + str(model_count) + '/Left/left'+ str(n) + '.bmp'
        #         filename_r = model_path +"model" + str(model_count) + '/Right/right'+ str(n) + '.bmp'  
        #         print('Captured model %d image\nPair %d ' %(model_count, n))
        #         n = 0
        #         model_count = model_count +1
        #     # cv.imwrite(filename_l, img_l)
        #     # cv.imwrite(filename_r, img_r)
        # elif choice & 0xFF == ord("n"):
        #     prj_count = prj_count +1
        #     print('Projecting pattern', prj_count)
        #     if prj_count == 10:
        #         prj_count = 0
        # elif choice & 0xFF == ord("q"):
        #     # StereoSystem.stop()
        #     # Robot.stop()
        #     break
