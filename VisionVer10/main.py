import cv2 as cv
from pypylon import pylon

import time, sys
import serial
from math import log

class RobotTask(threading.Thread):
    def __init__(self):
        global CurrentPos
        threading.Thread.__init__(self)
        self.con = serial.Serial(
        port = 'COM4',\
        baudrate=19200,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
            timeout=0)
        CurrentPos = self.RposC()
        time.sleep(3)
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
        out = 0x0130312C303030024d4f564c20302c2031302c20312c20
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
        global ImgArr, NowPos,CurrImg, CurrentPos
        self.con.write(b'\x05')
        received = 0
        distance = 20
        NowPos.append(CurrentPos)
        ImgArr.append(CurrImg)
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

    def run(self):
        global StartWelding,WeldPoint,CurrentPos
        while True:
            try:
                if StartWelding:
                    if CurrentPos[1] < -1635:
                        # CurrentPos[1] = CurrentPos[1] - 5
                        # command = CurrentPos
                        # self.movL(command)
                        CurrentPos[2] = CurrentPos[2] + 30
                        command = CurrentPos
                        self.movL(command)
                        self.con.close()
                        print("weld done")
                    command = np.concatenate((WeldPoint[0], CurrentPos[3:6]), 0)
                    del WeldPoint[0]
                    self.movL(command)
                    # print("lastY", lastY)
                else:
                    CurrentPos[1] = CurrentPos[1] - 12
                    command = CurrentPos
                    self.movL(command)
            except:
                pass
                # print("Robot error: ", sys.exc_info())
  
class CameraTask(threading.Thread):
    def __init__(self):
        self.Camvid = cv.VideoWriter('Camvid.avi', 
            cv.VideoWriter_fourcc(*'I420'), 10, (1928,1208))
        threading.Thread.__init__(self)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        self.camera.Width.SetValue(1928)
        self.camera.Height.SetValue(1208)
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImages) 
    def run(self):
        global CurrImg, newIMG
        while self.camera.IsGrabbing():
            t = time.time()
            try:
                grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    CurrImg = grabResult.Array
                    img = grabResult.Array
                    temp = cv.resize(img, (960, 520), interpolation = cv.INTER_AREA)
                    self.Camvid.write(cv.cvtColor(img,cv.COLOR_GRAY2RGB))
                    cv.imshow("Camera task", temp)
                    cv.waitKey(1)
                    newIMG = True
                    # print("fps: ", 1/(time.time() - t))
            except:
                print("Camera error: ",sys.exc_info())
    

class SoftwareTask(threading.Thread):
   def __init__(self):
      threading.Thread.__init__(self)
      self.vis = vision()
      self.softwarevid = cv.VideoWriter('softwarevid.avi', 
            cv.VideoWriter_fourcc(*'I420'), 1, (1928,1208))
      self.intrinsic = self.vis.intrinsic
      self.dist_coffs = self.vis.dist_coffs
      self.eye2hand = self.vis.eye2hand
      [self.a,self.b,self.c,self.d] = self.vis.plane
      self.fx = self.intrinsic[0][0]
      self.fy = self.intrinsic[1][1]
      self.cx = self.intrinsic[0][2]
      self.cy = self.intrinsic[1][2]
      self.tracker = cv.legacy.TrackerCSRT_create()
   def run(self):
        global ImgArr,NowPos,WeldPoint,StartWelding,CurrentPos, CurrImg, newIMG
        # bounding box size
        w = 150
        h = 360
        while True:
            if len(ImgArr) != 0:
                InitImg = ImgArr[0]
                pre = self.vis.Preprocessing(InitImg)
                thinned = cv.ximgproc.thinning(pre)
                center = self.vis.TripleLaser(thinned)
                _, _, imgpos = self.vis.CD(center)
                imgpos[1] = imgpos[1]
                # cv.circle(InitImg, (int(imgpos[0]),int(imgpos[1])), 10, 255, 10)
                # cv.rectangle(InitImg,(int(imgpos[0] - w/2), int(imgpos[1] - h/2)), (int(imgpos[0] + w/2), int(imgpos[1] + h/2)) ,255, 2)


                # initBB = (int(imgpos[0] - w5/2), int(imgpos[1] - h/2), w,h)
                initBB = (866, 243, 37, 355)
                imgpos[0] = initBB[0] + initBB[2] / 2
                imgpos[1] = initBB[1] + initBB[3] / 2
                self.tracker.init(InitImg, initBB)
                zlim = -437
                # calculate seam NowPos
                Zc = -self.d / (self.a/self.fx*(imgpos[0]-self.cx) + self.b/self.fy*(imgpos[1] - self.cy) + self.c)
                weldpoint2camera = np.array([[Zc/self.fx*(imgpos[0]-self.cx)], [Zc/self.fy*(imgpos[1]-self.cy)], [Zc] , [1]])
                tool2robot = self.vis.homogeneous(NowPos[0])
                weldpoint2robot = tool2robot.dot(self.eye2hand).dot(weldpoint2camera).flatten()[0:3]
                weldpoint2robot[0] = weldpoint2robot[0]
                weldpoint2robot[1] = weldpoint2robot[1]
                weldpoint2robot[2] = weldpoint2robot[2]
                if weldpoint2robot[2] < zlim:
                    weldpoint2robot[2] = zlim
                WeldPoint.append(np.round(weldpoint2robot,3))
                del NowPos[0]
                del ImgArr[0]
                # InitImg = cv.resize(InitImg, (960, 520), interpolation = cv.INTER_AREA)
                # cv.imshow("SoftwareTask", InitImg)
                cv.waitKey(1)
                break
        endpoint = -1900
        while True:
            # try:
            if newIMG == True and CurrentPos[1] > endpoint:
                    newIMG = False
                    # t = time.time()
                    # print("tracking")
                    img = CurrImg
                    (success, box) = self.tracker.update(CurrImg)
                    # print("tracking time: ", time.time() - t)
                    # if not success:
                        # print("track fail")
                    (x, y, w, h) = [int(v) for v in box]
                    cv.rectangle(img, (x, y), (x + w, y + h),255, 2)
                    cv.circle(img, (int(x + w / 2), int(y + h /2)), 5, 255, 5)
                    imgpos[0] = x + w / 2
                    imgpos[1] = y + h /2
                    self.softwarevid.write(cv.cvtColor(img,cv.COLOR_GRAY2RGB))
                    img = cv.resize(img, (960, 520), interpolation = cv.INTER_AREA)
                    # print("tracking successful")
                    cv.imshow("SoftwareTask", img)
                    cv.waitKey(3)
            # elif CurrentPos[1] < endpoint:
            #     print("track done")
            if len(ImgArr) != 0:
                Zc = -self.d / (self.a/self.fx*(imgpos[0]-self.cx) + self.b/self.fy*(imgpos[1] - self.cy) + self.c)
                weldpoint2camera = np.array([[Zc/self.fx*(imgpos[0]-self.cx)], [Zc/self.fy*(imgpos[1]-self.cy)], [Zc] , [1]])
                tool2robot = self.vis.homogeneous(NowPos[0])
                weldpoint2robot = tool2robot.dot(self.eye2hand).dot(weldpoint2camera).flatten()[0:3]
                weldpoint2robot[0] = weldpoint2robot[0]
                weldpoint2robot[1] = weldpoint2robot[1]
                weldpoint2robot[2] = weldpoint2robot[2]
                if weldpoint2robot[2] < zlim:
                    weldpoint2robot[2] = zlim
                WeldPoint.append(np.round(weldpoint2robot,3))
                print("distance: ", np.linalg.norm(WeldPoint[0][1] - CurrentPos[1]))
                if np.linalg.norm(WeldPoint[0][1] - CurrentPos[1]) < 10:
                    StartWelding = True
                    print("start true")
                # print("IMGARR LEN: ", len(WeldPoint))
                self.softwarevid.write(cv.cvtColor(ImgArr[0],cv.COLOR_GRAY2RGB))
                del NowPos[0]
                del ImgArr[0]
            # except:
            #     print("Software error: ", sys.exc_info())
            # finally:
            #     print("leng:", len(NowPos), len(ImgArr))

# Create new threads
ImgArr = []
StartWelding = False
NowPos = []
WeldPoint = []
# CurrImg = 0
# CurrentPos = np.array([0,0,0,0,0,0])

thread1 = CameraTask()
thread2 = RobotTask()
thread3 = SoftwareTask()
# Start new Threads
thread1.start()
thread2.start()
thread3.start()