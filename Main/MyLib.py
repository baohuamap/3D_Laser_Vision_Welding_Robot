'''
Library for 3D Laser Vision Welding Robot
    Vision and Robot Control modules

Author: Bao Hua
'''

import numpy as np
import cv2 as cv
from GlobalVariables import *
import socket
import time

CR = "\r"
CRLF = "\r\n"

def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

def savematrix(filename:str, matrix):
    with open(filename, 'w') as f:
        f.writelines('\t'.join(str(j) for j in i) + '\n' for i in matrix)

def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
    # note we also have to specify the type to retrieve otherwise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()
    cv_file.release()
    return [camera_matrix, dist_matrix]

def load_eye2hand(path):
     # FILE_STORAGE_READ
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
    eye2hand = cv_file.getNode("K1").mat()
    cv_file.release()
    return eye2hand

def load_laserplane(path):
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
    a, b, d = cv_file.getNode("P").mat()
    cv_file.release()
    return float(a), float(b), float(d)
class Vision():
    def __init__(self):
        self.intrinsic, self.dist_coffs = load_coefficients((camera_params))
        self.eye2hand = load_eye2hand((handeye_params))
        # coeffcients of laser plane: ax + by + cz + d = 0
        # a = 3.4980491606939381
        # b = 1.9111413571058655e-01
        c = -1
        # d  = 1.5376725252934193e+02
        a, b, d = load_laserplane((laserplane_params))
        self.plane = [a,b,c,d]
        # adjust according to sensor size
        self.rows = 1200
        self.cols = 1700

    def Preprocessing(self,img):
        newcameramtx, _ =cv.getOptimalNewCameraMatrix(self.intrinsic,self.dist_coffs,(self.rows,self.cols),1,(self.rows,self.cols))
        img = cv.undistort(img, self.intrinsic, self.dist_coffs, None, newcameramtx)
        # grayimg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)     # use when img is color image
        # blur = cv.GaussianBlur(grayimg,(7,7),0)
        blur = cv.GaussianBlur(img,(7,7),0)                 # use when img is gray image
        _, thresh = cv.threshold(blur,150,255,cv.THRESH_BINARY)
        closing = thresh
        del blur
        for _ in range(7):
            closing = cv.morphologyEx(closing, cv.MORPH_CLOSE, np.ones((7,7),np.uint8))
        del thresh
        return closing

    # http://tnuaa.nuaa.edu.cn/njhkhten/article/html/202004009
    def LaserCenter(self,img):
        center = np.zeros((self.rows,self.cols))
        # find the center point
        for x in range(self.cols):
            sum1 = 0.0
            sum2 = 0.0
            roi = np.where(img[:,x] == 255)
            if roi[0].size != 0:
                for y in roi[0]:
                    sum1 += y * img[y][x]
                    sum2 += img[y][x]
                center[int(sum1/sum2)][x] = 255
        return center

    def WeldseamCenter(self, img):
        weldseam_center = np.zeros((self.rows,self.cols))
        return weldseam_center

    def CrossLaser(self, img):
        colminval = self.rows/2
        mainline_img = np.zeros((self.rows,self.cols))
        roi = np.where(img.transpose() == 255)      #roi[0]: column, roi[1] row 
        for i in range(20, self.cols - 20):
            a = np.where(roi[0] == i)   # find pixel which value is 255 in each col
            if len(a[0]) == 1:
                mainline_img[roi[1][a[0][0]], roi[0][a[0][0]]] = 255
                colminval = roi[1][a[0][0]]
            elif len(a[0]) > 1:
                average_index = np.abs(roi[1][a[0]] - colminval).argmin()
                mainline_img[roi[1][a[0][average_index]], roi[0][a[0][average_index]]] = 255
                colminval = roi[1][a[0][average_index]]
        return mainline_img

    def TripleLaser(self,img):
        thinned = cv.ximgproc.thinning(img)
        mainline_img = np.zeros((self.rows,self.cols))
        roi = np.where(thinned.transpose() == 255)      #roi[0]: column, roi[1] row 
        a = np.where(roi[0] == 269)
        previous = np.array([0,0])
        current = np.array([0,0])
        for i in range(300, self.cols - 300):
            a = np.where(roi[0] == i)   # find pixel which value is 255 in each col
            ave = 0
            if len(a[0]) == 3:
                ave = (max(roi[1][a[0]]) + min(roi[1][a[0]]))/ 2
                average_index = np.abs(roi[1][a[0]] - ave).argmin()
                mainline_img[roi[1][a[0][1]],roi[0][a[0][1]]] = 255
                cv.circle(thinned, (roi[0][a[0][average_index]], roi[1][a[0][average_index]]), 5, 255, 1)
                previous = np.array([roi[0][a[0][average_index]],roi[1][a[0][average_index]]])

            elif len(a[0]) != 0:
                if ave != 0:
                    average_index = np.abs(roi[1][a[0]] - ave).argmin()
                    current = np.array([roi[0][a[0][average_index]],roi[1][a[0][average_index]]])
                    if np.linalg.norm(current[0:3] - previous[0:3]) < 10:
                        # cv.circle(thinned, (roi[0][a[0][average_index]],roi[1][a[0][average_index]]), 5, 255, 1)
                        mainline_img[roi[0][a[0][average_index]],roi[1][a[0][average_index]]] = 255
                        previous = current
        return mainline_img

    # Development of a real-time laser-based machine vision system to monitor and control welding processes 
    # by Wei Huang & Radovan Kovacevic
    def CD(self,img):
        feature1 = np.array([0, 0])
        feature2 = np.array([0, 0])
        roi = np.where(img.transpose() == 255)
        for i in range(28, len(roi[0]), 1):
            if roi[0][i+1] > roi[0][i] :
                feature1[0] = roi[0][i]
                feature1[1] = roi[1][i]
                feature2[0] = roi[0][i+1]
                feature2[1] = roi[1][i+1]
                break
        return feature1,feature2, (feature1 + feature2)/2
    
    def houghline(self, img):
        img = np.uint8(img)
        lines= cv.HoughLines(img, 1, np.pi/180.0, 150)

        a1 = -np.cos(lines[0][0][1])/np.sin(lines[0][0][1])
        b1 = lines[0][0][0]/np.sin(lines[0][0][1])
        a2 = -np.cos(lines[1][0][1])/np.sin(lines[1][0][1])
        b2 = lines[1][0][0]/np.sin(lines[1][0][1])

        x = (b2-b1)/(a1-a2)
        y = a1*x + b1
        if x > 4000:
            a2 = -np.cos(lines[2][0][1])/np.sin(lines[2][0][1])
            b2 = lines[2][0][0]/np.sin(lines[2][0][1])
            x = (b2-b1)/(a1-a2)
            y = a1*x + b1
        return [x,y]
        
    # Roll Pitch Yaw to rotation matrix
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

    # homogeneous matrix create
    def homogeneous(self,posc):
        R, t = self.posc2Rt(posc)
        transformation = np.zeros(shape=(4,4))
        for i in range(3):
            transformation[i][3] = t[i][0]
            for j in range(3):
                transformation[i][j] = R[i][j]
        transformation[3][3] = 1
        return transformation

    # Rotation matrix to Roll Pitch Yaw
    def mtrx2RPY(mtrx):
        from math import atan2,sqrt,pi
        Rz=atan2(mtrx[1,0],mtrx[0,0]);
        Ry=atan2(-mtrx[2,0],sqrt(mtrx[2,1]**2+mtrx[2,2]**2))
        Rx=atan2(mtrx[2,1],mtrx[2,2]);
        return [Rx*180/pi, Ry*180/pi, Rz*180/pi]

    def PlaneFitting(Point):
        x_square = 0
        xy = 0
        x = 0
        y_square = 0
        y = 0
        N = 0
        xz = 0
        yz = 0
        z = 0
        for i in range(len(Point)):
            x_square = x_square + Point[i][0][0]**2
            xy = xy + Point[i][0][0] * Point[i][1][0]
            x = x + Point[i][0][0]
            y_square = y_square + Point[i][1][0]**2
            y = y + Point[i][1][0]
            N = N + 1
            xz = xz + Point[i][0][0]*Point[i][2][0]
            yz = yz + Point[i][1][0]*Point[i][2][0]
            z = z + Point[i][2][0]
        M = np.array([[x_square,  xy, x], [xy, y_square, y],[x, y, N]])
        plane = np.linalg.inv(M).dot(np.array([[xz], [yz], [z]]))
        return plane

    def FeatureExtraction(img):
        img = np.uint8(img)
        lines= cv.HoughLines(img, 1, np.pi/180.0, 100)
        a1 = -np.cos(lines[0][0][1])/np.sin(lines[0][0][1])
        b1 = lines[0][0][0]/np.sin(lines[0][0][1])
        cv.line(img,(1000,int(a1*1000+b1)),(3000,int(a1*3000+b1)),255,2)
        a2 = -np.cos(lines[1][0][1])/np.sin(lines[1][0][1])
        b2 = lines[1][0][0]/np.sin(lines[1][0][1])
        # print("line2: ",a2,b2)
        cv.line(img,(1000,int(a2*1000+b2)),(3000,int(a2*3000+b2)),255,2)
        x = (b2-b1)/(a1-a2)
        y = a1*x + b1
        cv.circle(img, (int(x),int(y)), 20, [255,0,0], 10)
        return [x, y]
        
    def create_objpoint(self):
        square_size = 15
        objp = np.zeros((7*10,3), np.float32)  # checker size 8x11
        objp[:,:2] = np.mgrid[0:10,0:7].T.reshape(-1,2)*square_size
        return objp

class Yaskawa():
    def __init__(self):
        self.ip_address = '192.168.255.2'
        self.port = 80
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.settimeout(5)
    
    def utf8len(self, inputString):
        return len(inputString.encode('utf-8'))

    def command_data_length(self, command):
        if len(command) == 0:
            return 0
        else:
            return self.utf8len(command + CR)

    def pos2command(self, pos):
        command = ""
        for i in range(len(pos)):
            command += str(pos[i]) + ","
        # command = command[:-1]
        return command

    def read_pos_from_txt(self, pos):
        trajectory = open(self.robot_path, "r")
        for i, line in enumerate(trajectory):
            if i == pos - 1:
                command = line.rstrip()
        # self.CurrentPos = np.fromstring(command, dtype = float, sep=',')
        return command

    def StartRequest(self):
        self.client.connect((self.ip_address, self.port))
        #START request
        startRequest = "CONNECT Robot_access Keep-Alive:-1" + CRLF
        self.client.send(startRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        startResponse = repr(response)
        # print(startResponse)
        if 'OK: DX Information Server' not in startResponse:
            self.client.close()
            print('[E] Command start request response to DX100 is not successful!')

    def Stop(self):
        self.client.close()

    def MovJ(self, command):
        #COMMAND request
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "MOVJ" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        # print(commandResponse)
        if ('OK: ' + "MOVJ" not in commandResponse):
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = command + (CR if len(command) > 0 else '')
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
        return commandDataResponse

    def RposC(self):
        #COMMAND request
        command = "1, 0"
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "RPOSC" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ('OK: ' + "RPOSC" not in commandResponse):
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = command + CR
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
        time.sleep(0.01)
        current_pos = np.fromstring(commandDataResponse[2:50], dtype=float, sep=',')
        return current_pos

    def MovL(self, command):
        #COMMAND request
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "MOVL" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ('OK: ' + "MOVL" not in commandResponse):
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = command + (CR if len(command) > 0 else '')
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
        return commandDataResponse
        

    def ArcOn(self):
        pass 

    def ArcOff(self):
        pass   