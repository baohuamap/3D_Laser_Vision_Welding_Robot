import os
import numpy as np
import cv2 as cv
from pypylon import pylon
from GlobalVariables import *
import math
from math import log
import threading
os.environ["PYLON_CAMEMU"] = "3"
import serial
import time
from save_load_lib import savematrix
import glob

check_left = checkerboard_path_left
check_right = checkerboard_path_right

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
    def __init__(self,key):
        self.key = key
        self.m = 0
        self.stopped = False
        self.con = serial.Serial(
        port = 'COM3',\
        baudrate=19200,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
            timeout=0) #Change COM port
        self.CurrentPos = self.RposC()
        self.model_count = 0
        # time.sleep(3)
    def __bbc(self,input):
        sum = 0
        while input != 0:
            char = input%0x100
            sum = sum +char
            input = input//0x100
            result = (sum - 0x01)//0x100 + (sum - 0x01)%0x100*0x100
        return result

    def __SplitData(self, received):
        string = []
        pos = np.zeros(6)
        while received != 0:
            char = received%0x100
            received = received//0x100
            string.insert(0, char - 48)         # convert ascii to number
        end = string.index(-46) + 1
        for k in range(6):
            string = string[end:-1]
            end = string.index(-4) + 1
            element = string[0: end - 1] 
            if -3 not in element:
                sign = 1
            else:
                sign = -1
                element.remove(-3)
            dot = element.index(-2)
            element.remove(-2)
            number = 0
            for i in range(len(element)):
                number = number + sign * element[i] * 10**(dot - i - 1)
            pos[k] = number
        return pos
    def __ReadUntil(self, bytes):
        line = bytearray()
        c = b'\x00'
        while c.find(bytes) == -1:
            c = self.con.readline()
            if c:
                line = line + c
        time.sleep(0.02)
        return line
      
    def RposC(self):
        self.con.write(b'\x05')
        received = 0
        while received != 4:
            while self.con.inWaiting():
                received = int.from_bytes(self.con.readline(), byteorder='big', signed=False)
                if received == 4144: #to Hex = 1030 = ACK0
                    self.con.write(b'\x0101,000\x02RPOSC 1, 0\x0D\x03\x83\x03'), # recieved ACK0 -> send RPOSC command
                elif received == 4145:
                    self.con.write(b'\x04')
                elif received == 5:   # received ENQ
                    self.con.write(b'\x100')    #send ACK0
                    data = int.from_bytes(self.__ReadUntil(b'\x03'), byteorder='big', signed=False)
                    result = self.__SplitData(data)
                    self.con.write(b'\x101')    #send ACK1
        time.sleep(0.02)
        return np.round(result,3)

    def __CombineData(self,position):
        input = str(position[0]) + ', ' + str(position[1]) + ', ' + str(position[2]) + ', ' + str(position[3]) + ', ' + str(position[4]) + ', ' + str(position[5])
        out = 0x0130312C303030024d4f564c20302c2034302c20312c20
        for i in range(len(input)):
            out = out * 0x100 + ord(input[i])
        for _ in range(8):
            out = out * 0x1000000 + 0x2C2030
        out = out * 0x100 + 0x0D03
        return out

    def __bytes_needed(self,number): 
        return int(log(number, 256)) + 1

    def hold(self, data):
        self.con.write(b'\x05')
        received = 0
        while received != 4:
            while self.con.inWaiting():
                received = int.from_bytes(self.con.readline(), byteorder='big', signed=False)
                if received == 4144 and data == 1:
                    self.con.write(b'\x01\x30\x31\x2C\x30\x30\x30\x02\x48\x4f\x4c\x44\x20\x31\x0D\x03\xA7\x02')
                if received == 4144 and data == 0:
                    self.con.write(b'\x01\x30\x31\x2C\x30\x30\x30\x02\x48\x4f\x4c\x44\x20\x30\x0D\x03\xA6\x02')
                elif received == 4145:
                    self.con.write(b'\x04')
                elif received == 5:   # received EQN
                    self.con.write(b'\x100')    #send ACK0
                    _ = int.from_bytes(self.__ReadUntil(b'\x03'), byteorder='big', signed=False)
                    self.con.write(b'\x101')    #send ACK1               
    def __RPY2mtrx(self,posc):
        from math import pi,sin,cos
        Rx = posc[3]*pi/180
        Ry = posc[4]*pi/180
        Rz = posc[5]*pi/180
        return np.array([[cos(Rz)*cos(Ry), cos(Rz)*sin(Ry)*sin(Rx) - sin(Rz)*cos(Rx), sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx)],
        [sin(Rz)*cos(Ry), cos(Rz)*cos(Rx) + sin(Rz)*sin(Ry)*sin(Rx), sin(Rz)*sin(Ry)*cos(Rx)-cos(Rz)*sin(Rx)],
        [-sin(Ry), cos(Ry)*sin(Rx), cos(Ry)*cos(Rx)]])
    def posc2Rt(self,posc):
        R = self.__RPY2mtrx(posc)
        t = np.array([posc[0], posc[1] ,posc[2]]).reshape(3,1)
        return R,t    
    def movL(self,position):
        self.con.write(b'\x05')
        received = 0
        distance = 20
        while distance > 8:
            while self.con.inWaiting():
                received = int.from_bytes(self.con.readline(), byteorder='big', signed=False)
                if received == 4144:
                    command = self.__CombineData(position)
                    command = command * 0x10000 + self.__bbc(command)
                    self.con.write(command.to_bytes(self.__bytes_needed(command), 'big'))
                elif received == 4145:
                    self.con.write(b'\x04')
                elif received == 5:   # received EQN
                    time.sleep(0.02)
                    self.con.write(b'\x100')    #send ACK0
                    _ = int.from_bytes(self.__ReadUntil(b'\x03'), byteorder='big', signed=False)
                    time.sleep(0.02)
                    self.con.write(b'\x101')    #send ACK1
                elif received == 4:
                    while distance > 7:
                        time.sleep(0.02)
                        CurrentPos = self.RposC()
                        distance = np.linalg.norm(position[0:3] - CurrentPos[0:3])
    def read(self):
        self.CurrentPos = self.RposC()
        R, t = self.posc2Rt(self.CurrentPos)
        return R,t
    def save(self,path,mtx):
        cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
        cv_file.write("K1", mtx)
        cv_file.release()

    def _save_model_poses(self,H, num):
        cv_file = cv.FileStorage('Model_poses'+str(num)+'.txt', cv.FILE_STORAGE_WRITE)
        cv_file.write("K1", H)
        cv_file.release()
    def move(self):
        desire_pos = np.array([[490, 0, 375,  180,  -90,  0],
            [288.623, -23.641, 264.455, 40.59, 39.84, 46.67],
            [302.132, -21.581, 338.562, 80.5, 52.13, 78.31],
            [604.408, -47.674, 266.534, 154.47, 38.33, 133.99],
            [522.621, -282.996, 256.576, 162.24, 30.75, 113.8],
            [411.075, -417.572, 193.769, 163.6, 16.01, 78.68],
            [524.372, 210.354, 188.609, 177.9, 11.49, 149.35]])     
        self.movL(desire_pos[self.m])
        self.m = self.m +1
    def _toHomo(self,R,t):
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
        while not self.stopped:
            if self.key &0xFF == ord('c'):
                R,t = self.read()
                print('\nCurrent Pos is\n')
                print('R:\n',R)
                print('T:\n',t)
                R_end2base.append(R)
                T_end2base.append(t)
                time.sleep(0.5)
            elif self.key &0xFF == ord('s'):
                RR= np.array(R_end2base)
                TT = np.array(T_end2base)
                # RR = np.concatenate( R_end2base, axis=0 )
                # TT = np.concatenate( T_end2base, axis=0 )
                self.save(R_path,RR)
                self.save(t_path,TT)
                print('Saved')
                time.sleep(3)
            elif self.key &0xFF == ord('p'):
                pattern_count= pattern_count+1
                if pattern_count == 10:
                    R,t = self.read()
                    H = self._toHomo(R,t)
                    H_end2base_model.append(H)
                    H = np.array(H_end2base_model)
                    self._save_model_poses(H,self.model_count)
                    print('Record model position ',self.model_count)
                    pattern_count = 0
                    time.sleep(1)

    def stop(self):
        self.stopped = True
    def start(self):
        threading.Thread(target=self.process, args=()).start()


if __name__ == "__main__":
    choice = 1

    StereoSystem = BaslerCam(choice)
    # Robot = RobotTask(choice)
    StereoSystem.start()
    # Robot.start()
    t = 2
    prj_im_list = glob.glob(projecting_pattern +'*.jpg')
    
    check_count = 0
    prj_count = 0
    n = 0
    model_count = 5
    while True:
        prj_im = cv.imread(prj_im_list[prj_count])
        img_l = StereoSystem.im_l
        img_r = StereoSystem.im_r
        frame = np.hstack([cv.resize(img_l,(720,480)),cv.resize(img_r,(720,480))]) 
        cv.imshow('Frame',frame)
        cv.imshow('Pattern',cv.resize(prj_im,(int(1920/t),int(1080/t))))
        choice = cv.waitKey(70)
        # Robot.key = choice
        # Robot.model_count = model_count
        if choice & 0xFF == ord("c"):
            check_count = check_count +1
            # print(count)
            # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
            if check_count <10:
                filename_l = check_left +"left0" + str(check_count) +'.bmp'
                filename_r = check_right +"right0"+ str(check_count) +'.bmp'
            else:
                filename_l = check_left +"left" + str(check_count) +'.bmp'
                filename_r = check_right +"right"+ str(check_count) +'.bmp'
            cv.imwrite(filename_l, img_l)
            cv.imwrite(filename_r, img_r)
            print('Captured checkerboard image\nPair %d ' %(check_count))
        elif choice & 0xFF == ord("p"):
            n = n+1
            if n <10:
                filename_l = model_path +"model" + str(model_count) + '/Left/left0'+ str(n) + '.bmp'
                filename_r = model_path +"model" + str(model_count) + '/Right/right0'+ str(n) + '.bmp'
                print('Captured model %d image\nPair %d ' %(model_count, n))
            if n ==10:
                filename_l = model_path +"model" + str(model_count) + '/Left/left'+ str(n) + '.bmp'
                filename_r = model_path +"model" + str(model_count) + '/Right/right'+ str(n) + '.bmp'  
                print('Captured model %d image\nPair %d ' %(model_count, n))
                n = 0
                model_count = model_count +1
            cv.imwrite(filename_l, img_l)
            cv.imwrite(filename_r, img_r)
        elif choice & 0xFF == ord("n"):
            prj_count = prj_count +1
            print('Projecting pattern', prj_count)
            if prj_count == 10:
                prj_count = 0
        elif choice & 0xFF == ord("q"):
            StereoSystem.stop()
            # Robot.stop()
            break
