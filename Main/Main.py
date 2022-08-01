import cv2 as cv
from pypylon import pylon
from MyLib import vision
import numpy as np
import threading
import time, sys
from GlobalVariables import *
from math import log
import socket

CR = "\r"
CRLF = "\r\n"

weld_start_path = welding_trajectory

class RobotTask(threading.Thread):
    def __init__(self):
        global CurrentPos
        threading.Thread.__init__(self)
        self.ip_address = "192.168.255.2"
        self.port = 80
        CurrentPos = self.RposC()
        self.stop = False
        time.sleep(3)

    def utf8len(self,inputString):
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
        trajectory = open(self.trajectory_path, "r")
        for i, line in enumerate(trajectory):
            if i == pos - 1:
                command = line.rstrip()
        self.CurrentPos = np.fromstring(command, dtype=float, sep=',')
        return command
    
    def MovJ(self, command):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        client.connect((self.ip_address, self.port))
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
        # print(commandResponse)
        if ('OK: ' + "MOVJ" not in commandResponse):
            client.close()
            print('[E] Command request response to DX100 is not successful!')
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

    def RposC(self):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        client.connect((self.ip_address, self.port))
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
        commandLength = self.command_data_length("1, 0")
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
            #COMMAND DATA request
            commandDataRequest = "1, 0"
            client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = client.recv(4096)
            commandDataResponse = repr(response)
            # print(commandDataResponse)
            if commandDataResponse:
                client.close()
        time.sleep(0.01)
        current_pos = np.fromstring(commandDataResponse[2:50], dtype=float, sep=',')
        return current_pos

    def MovL(self, command):
        global ImgArr, NowPos,CurrImg, CurrentPos
        distance = 20
        NowPos.append(CurrentPos)
        ImgArr.append(CurrImg)

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        client.connect((self.ip_address, self.port))
        startRequest = "CONNECT Robot_access" + CRLF
        client.send(startRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096)      #4096: buffer size
        startResponse = repr(response)
        if 'OK: DX Information Server' not in startResponse:
            client.close()
            print('[E] Command start request response to DX100 is not successful!')
            return
        #COMMAND request
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "MOVL" + " " + str(commandLength) + CRLF
        client.send(commandRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        # print(commandResponse)
        if ('OK: ' + "MOVL" not in commandResponse):
            client.close()
            print('[E] Command request response to DX100 is not successful!')
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
                CurrentPos = self.RposC()
                # distance = np.linalg.norm(position[0:3] - CurrentPos[0:3])
        time.sleep(0.01)

    def ArcOn(self):
        pass 

    def ArcOff(self):
        pass        

    # def MovL(self,position):
        # global ImgArr, NowPos,CurrImg, CurrentPos
        # self.con.write(b'\x05')
        # received = 0
        # distance = 20
        # NowPos.append(CurrentPos)
        # ImgArr.append(CurrImg)
        # while distance > 8:
        #     while self.con.inWaiting():
        #         received = int.from_bytes(self.con.readline(), byteorder='big', signed=False)
        #         if received == 4144:
        #             command = self.__CombineData(position)
        #             command = command * 0x10000 + self.__bbc(command)
        #             self.con.write(command.to_bytes(self.__bytes_needed(command), 'big'))
        #         elif received == 4145:
        #             self.con.write(b'\x04')
        #         elif received == 5:   # received EQN
        #             time.sleep(0.02)
        #             self.con.write(b'\x100')    #send ACK0
        #             _ = int.from_bytes(self.__ReadUntil(b'\x03'), byteorder='big', signed=False)
        #             time.sleep(0.02)
        #             self.con.write(b'\x101')    #send ACK1
        #         elif received == 4:
        #             while distance > 8:
        #                 time.sleep(0.02)
        #                 CurrentPos = self.RposC()
        #                 distance = np.linalg.norm(position[0:3] - CurrentPos[0:3])

    def run(self):
        global StartWelding,WeldPoint,CurrentPos
        lastY = CurrentPos[1]
        while True:
            try:
                if StartWelding:
                    self.ArcOn()
                    if CurrentPos[1] < -1530:
                        # CurrentPos[1] = CurrentPos[1] - 5
                        # command = CurrentPos
                        # self.movL(command)
                        CurrentPos[2] = CurrentPos[2] + 30
                        command = CurrentPos
                        self.MovL(command)
                        self.ArcOff()
                        print("weld done")
                        WeldDone = True
                    if WeldPoint[0][1] > lastY:
                        WeldPoint[0][1] = lastY - 15
                    command = np.concatenate((WeldPoint[0], CurrentPos[3:6]), 0)
                    lastY = WeldPoint[0][1]
                    del WeldPoint[0]
                    self.MovL(command)
                    # print("lastY", lastY)
                else:
                    CurrentPos[1] = CurrentPos[1] - 12
                    command = CurrentPos
                    self.MovL(command)
            except:
                print("Robot error: ", sys.exc_info())

class CameraTask(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        self.camera.Width.SetValue(1928)
        self.camera.Height.SetValue(1208)
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImages) 
        
    def run(self):
        global CurrImg
        while self.camera.IsGrabbing():
            t = time.time()
            try:
                grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    CurrImg = grabResult.Array
                    img = grabResult.Array
                    temp = cv.resize(img, (960, 520), interpolation = cv.INTER_AREA)
                    cv.imshow("Camera task", temp)
                    cv.waitKey(1)
                    print("fps: ", 1/(time.time() - t))
            except:
                print("Camera error: ",sys.exc_info())
    
class SoftwareTask(threading.Thread):
   def __init__(self):
      threading.Thread.__init__(self)
      self.vis = vision()
      self.intrinsic = self.vis.intrinsic
      self.dist_coffs = self.vis.dist_coffs
      self.eye2hand = self.vis.eye2hand
      [self.a,self.b,self.c,self.d] = self.vis.plane
      self.fx = self.intrinsic[0][0]
      self.fy = self.intrinsic[1][1]
      self.cx = self.intrinsic[0][2]
      self.cy = self.intrinsic[1][2]

   def run(self):
        global ImgArr,NowPos,WeldPoint,StartWelding,CurrentPos
        # bounding box size
        w = 500
        h = 150
        while True:
            if len(ImgArr) != 0:
                InitImg = ImgArr[0]
                pre = self.vis.Preprocessing(InitImg)
                center = self.vis.LaserCenter(pre)
                _, _, imgpos = self.vis.CD(center)
                cv.circle(InitImg, (int(imgpos[0]),int(imgpos[1])), 50, 255, 10)
                cv.rectangle(InitImg,(int(imgpos[0] - w/2), int(imgpos[1] - h/2)), (int(imgpos[0] + w/2), int(imgpos[1] + h/2)) ,255, 2)

                initBB = (int(imgpos[0] - w/2), int(imgpos[1] - h/2), w,h)
                tracker = cv.legacy.TrackerCSRT_create()
                tracker.init(InitImg, initBB)

                # calculate seam NowPos
                Zc = -self.d / (self.a/self.fx*(imgpos[0]-self.cx) + self.b/self.fy*(imgpos[1] - self.cy) + self.c)
                weldpoint2camera = np.array([[Zc/self.fx*(imgpos[0]-self.cx)], [Zc/self.fy*(imgpos[1]-self.cy)], [Zc] , [1]])
                tool2robot = self.vis.homogeneous(NowPos[0])
                weldpoint2robot = tool2robot.dot(self.eye2hand).dot(weldpoint2camera).flatten()[0:3]
                weldpoint2robot[0] = weldpoint2robot[0]
                weldpoint2robot[1] = weldpoint2robot[1] + 10
                weldpoint2robot[2] = weldpoint2robot[2] + 50
                if weldpoint2robot[2] < -532:
                    weldpoint2robot[2] = -532
                WeldPoint.append(np.round(weldpoint2robot,3))
                del NowPos[0]
                del ImgArr[0]
                InitImg = cv.resize(InitImg, (960, 520), interpolation = cv.INTER_AREA)
                cv.imshow("SoftwareTask", InitImg)
                cv.waitKey(1)
                break
        while True:
            try:
                if len(ImgArr) != 0:
                    img = ImgArr[0]
                    t = time.time()
                    print("tracking")
                    (success, box) = tracker.update(img)
                    print("tracking time: ", time.time() - t)
                    if not success:
                        print("track fail")
                    (x, y, w, h) = [int(v) for v in box]
                    cv.rectangle(img, (x, y), (x + w, y + h),255, 2)
                    cv.circle(img, (int(x + w / 2), int(y + h /2)), 5, 255, 5)
                    imgpos[0] = x + w / 2
                    imgpos[1] = y + h /2
                    # calculate seam NowPos
                    Zc = -self.d / (self.a/self.fx*(imgpos[0]-self.cx) + self.b/self.fy*(imgpos[1] - self.cy) + self.c)
                    weldpoint2camera = np.array([[Zc/self.fx*(imgpos[0]-self.cx)], [Zc/self.fy*(imgpos[1]-self.cy)], [Zc] , [1]])
                    tool2robot = self.vis.homogeneous(NowPos[0])
                    weldpoint2robot = tool2robot.dot(self.eye2hand).dot(weldpoint2camera).flatten()[0:3]
                    weldpoint2robot[0] = weldpoint2robot[0]
                    weldpoint2robot[1] = weldpoint2robot[1] + 10
                    weldpoint2robot[2] = weldpoint2robot[2] + 50
                    if weldpoint2robot[2] < -532:
                        weldpoint2robot[2] = -532
                    WeldPoint.append(np.round(weldpoint2robot,3))
                    print("distance: ", np.linalg.norm(WeldPoint[0][1] - CurrentPos[1]))
                    if np.linalg.norm(WeldPoint[0][1] - CurrentPos[1]) < 10:
                        StartWelding = True
                        print("start true")
                    print("IMGARR LEN: ", len(WeldPoint))
                    del NowPos[0]
                    del ImgArr[0]
                    img = cv.resize(img, (960, 520), interpolation = cv.INTER_AREA)
                    print("tracking successful")
                    cv.imshow("SoftwareTask", img)
                    cv.waitKey(1)
            except:
                print("Software error: ", sys.exc_info())
            finally:
                print("leng:", len(NowPos), len(ImgArr))

# # Create new threads
ImgArr = []
StartWelding = False
WeldDone = False
NowPos = []
WeldPoint = []
# # CurrImg = 0
# # CurrentPos = np.array([0,0,0,0,0,0])

thread1 = CameraTask()
thread2 = RobotTask()
thread3 = SoftwareTask()

# Start new Threads
thread1.start()
thread2.start()
thread3.start()