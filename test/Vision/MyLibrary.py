import sys
import cv2 as cv
import numpy as np
# import serial
import time
from math import log

class vision():
    def __init__(self):
        self.intrinsic=np.array([[1.43613960e+03,0.00000000e+00,9.45116586e+02],
                                [0.00000000e+00,1.43648424e+03,5.83160320e+02],
                                [0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.dist_coffs=np.array([[-8.76138852e-02, 2.07436397e-02, 1.09728038e-03, -3.02686811e-05, 3.81461021e-01]])
        # eye to hand coordinatex`
        self.eye2hand = np.array([[-8.74782056e-01, -3.79300492e-01, -3.01475524e-01,  9.29334550e+01],
                                [ 4.84503838e-01, -6.89318581e-01, -5.38605538e-01,  2.08188779e+02],
                                [-3.51933429e-03, -6.17228508e-01,  7.86776069e-01, -1.14571713e+02],
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        # coeffcients of laser plane: ax + by + cz + d = 0
        a = 1.49391238e-01
        b = 1.31427483e+00
        c = -1
        d  = 2.32748012e+02
        self.plane = [a,b,c,d]
        # adjust according to img
        # self.rows = 2748
        # self.cols = 3840
        self.rows = 1200
        self.cols = 1732
        # self.rows = 720
        # self.cols = 1080

    def Preprocessing(self,img):
        newcameramtx, _ =cv.getOptimalNewCameraMatrix(self.intrinsic,self.dist_coffs,(self.rows,self.cols),1,(self.rows,self.cols))
        # undistort
        # img = cv.undistort(img, self.intrinsic, self.dist_coffs, None, newcameramtx)
        # grayimg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(img,(7,7),0)
        _, thresh = cv.threshold(blur,100,255,cv.THRESH_BINARY)
        closing = thresh
        del blur
        for _ in range(5):
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
        # cv.imshow("temp", thinned)
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

    #Development of a real-time laser-based machine vision system to monitor and control welding processes 
    # by Wei Huang & Radovan Kovacevic
    def CD(self,img):
        feature1 = np.array([0, 0])
        feature2 = np.array([0, 0])
        roi = np.where(img.transpose() == 255)
        for i in range(105, len(roi[0]), 1):
            if roi[0][i+1] > roi[0][i] :
                feature1[0] = roi[0][i]
                feature1[1] = roi[1][i]
                feature2[0] = roi[0][i+1]
                feature2[1] = roi[1][i+1]
                break
        # cv.circle(img, (feature1[0], feature1[1]), 10, 255, 5) 
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
        
    def CalibParams(self):
        square_size = 15
        objp = np.zeros((7*10,3), np.float32)  # checker size 8x11
        objp[:,:2] = np.mgrid[0:10,0:7].T.reshape(-1,2)*square_size
        return objp


class yaskawa():
    def __init__(self,port_,baud_ = 19200):
        self.con = serial.Serial(
        port = port_,\
        baudrate=baud_,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
            timeout=0)

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
                if received == 4144:
                    self.con.write(b'\x0101,000\x02RPOSC 1, 0\x0D\x03\x83\x03'), # recieved ACK0 -> send RPOSC command
                elif received == 4145:
                    self.con.write(b'\x04')
                elif received == 5:   # received EQN
                    self.con.write(b'\x100')    #send ACK0
                    data = int.from_bytes(self.__ReadUntil(b'\x03'), byteorder='big', signed=False)
                    result = self.__SplitData(data)
                    self.con.write(b'\x101')    #send ACK1
        time.sleep(0.02)
        return np.round(result,3)

    def __CombineData(self,position):
        input = str(position[0]) + ', ' + str(position[1]) + ', ' + str(position[2]) + ', ' + str(position[3]) + ', ' + str(position[4]) + ', ' + str(position[5])
        out = 0x0130312C303030024d4f564c20302c2032302c20312c20
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
    def movL(self,position):
        self.con.write(b'\x05')
        received = 0
        distance = 20
        while distance > 10:
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
                    while distance > 5:
                        time.sleep(0.02)
                        currP = self.RposC()
                        distance = np.linalg.norm(position[0:3] - currP[0:3]) # calculate distance respectively to x axis
                        
