import cv2 as cv
from pypylon import pylon
from MyLib import Vision, Yaskawa
import numpy as np
import threading
import time, sys
from GlobalVariables import *

# --- Choose the Welding_HomePos ---
# Welding_HomePos = [928, 19, -460, -180, 0, 0] # butt welding linear
# Welding_HomePos = [1037, -16, -430, -135, 0, 0] # fillet welding 1
# Welding_HomePos = [1037, 39, -430, 135, 0, 0] # fillet welding 2
Welding_HomePos = [910, -50.5, -460, -180, 0, 0] # butt welding spline
# Welding_HomePos = [924, -211, -450, -180, 0, 0] # butt welding zic zac

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
        command = "0, 5, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
        return command

    def pos2Movjcommand(self, pos):
        position = ""
        for i in pos:
            position += str(i) + ", "
        command = "8, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
        return command

    def homing(self):
        command = self.pos2Movjcommand(Welding_HomePos)
        self.robot.MovJ(command)

    def move(self, position):
        global ImgArr, NowPos,CurrImg, CurrentPos
        distance = 10
        NowPos.append(CurrentPos)
        ImgArr.append(CurrImg)
        while distance > 5:
            command = self.pos2Movlcommand(position)    
            reps = self.robot.MovL(command)
            time.sleep(0.01)
            if not "ERROR" in reps:
                while distance > 5:
                    time.sleep(0.01)
                    CurrentPos = self.robot.RposC()
                    distance = np.linalg.norm(position[0:3] - CurrentPos[0:3])
                    # print(distance)
            else:
                pass
        # command = self.pos2Movlcommand(position)
        # reps = self.robot.MovL(command)
        # if not "ERROR" in reps:
        #     CurrentPos = self.robot.RposC()

    def stop(self):
        global StopProcess, StartWelding
        self.robot.Stop()
        StopProcess = True
        StartWelding = False
        print("weld done")

    def run(self):
        global StartWelding, WeldPoint, CurrentPos, StopProcess
        lastX = CurrentPos[0]
        while not StopProcess:
            try:
                if StartWelding:
                    if CurrentPos[0] > 1269:
                        CurrentPos[2] = CurrentPos[2] + 50
                        pos = CurrentPos
                        self.robot.MovJ(self.pos2Movjcommand(pos))
                        time.sleep(2)
                        self.stop()
                        StartWelding = False
                    if WeldPoint[0][0] < lastX:
                        WeldPoint[0][0] = lastX + 10
                    pos = np.concatenate((WeldPoint[0], CurrentPos[3:6]), 0)
                    lastX = WeldPoint[0][0]
                    del WeldPoint[0]
                    self.move(pos)
                else:
                    CurrentPos[0] = CurrentPos[0] + 10
                    pos = CurrentPos
                    self.move(pos)
            except:
                print("Robot error: ", sys.exc_info())
                pass
            # finally:
                # pass

class CameraTask(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        self.camera.Width.SetValue(1700)
        self.camera.Height.SetValue(1200)
        self.camera.OffsetX.SetValue(8)
        self.camera.OffsetY.SetValue(8)
        self.camera.ExposureTimeAbs = 10000
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
        global ImgArr, NowPos, WeldPoint, StartWelding, CurrentPos, StopProcess
        # bounding box size
        w = 100
        h = 100
        InitImg = ImgArr[0]
        # feature extraction
        pre = self.vis.Preprocessing(InitImg)
        thinned = cv.ximgproc.thinning(pre, thinningType=cv.ximgproc.THINNING_ZHANGSUEN)
        center = self.vis.LaserCenter(thinned)
        _, _, imgpos = self.vis.CD(center)
        # imgpos = [818, 771] # weld butt zic zac
        # imgpos = [835, 528] # weld butt
        imgpos = [918,638] # weld butt SPLINE
        # imgpos = [925, 623] # weld fillet
        cv.circle(InitImg, (int(imgpos[0]),int(imgpos[1])), 5, 255, 5)
        cv.rectangle(InitImg,(int(imgpos[0] - w/2), int(imgpos[1] - h/2)), (int(imgpos[0] + w/2), int(imgpos[1] + h/2)) ,255, 2)
        # tracker initialization
        initBB = (int(imgpos[0] - w/2), int(imgpos[1] - h/2), w, h)
        tracker = cv.legacy.TrackerCSRT_create()
        tracker.init(InitImg, initBB)
        # calculate seam NowPos
        Zc                 = -self.d / (self.a/self.fx*(imgpos[0]-self.cx) + self.b/self.fy*(imgpos[1] - self.cy) + self.c)
        weldpoint2camera   = np.array([[Zc/self.fx*(imgpos[0]-self.cx)], [Zc/self.fy*(imgpos[1]-self.cy)], [Zc] , [1]])
        tool2robot         = self.vis.homogeneous( NowPos[0])
        weldpoint2robot    = tool2robot.dot(self.eye2hand).dot(weldpoint2camera).flatten()[0:3]
        print(weldpoint2robot)
        weldpoint2robot[0] = weldpoint2robot[0] 
        weldpoint2robot[1] = weldpoint2robot[1] - 15
        weldpoint2robot[2] = weldpoint2robot[2] - 5
        if weldpoint2robot[2] < -485:
            weldpoint2robot[2] = -485
        WeldPoint.append(np.round(weldpoint2robot,3))
        del NowPos[0]
        del ImgArr[0]
        while not StopProcess:
            try:
                if len(ImgArr) != 0:
                    img = ImgArr[0]
                    # tracking weld seam
                    (success, box) = tracker.update(img)
                    if success:
                        (x, y, w, h) = [int(v) for v in box]
                        cv.rectangle(img, (x, y), (x + w, y + h),255, 2)
                        cv.circle(img, (int(x + w / 2), int(y + h /2)), 5, 255, 5)
                    else:
                        print("track fail")
                    imgpos[0] = x + w/2
                    imgpos[1] = y + h/2
                    # calculate seam NowPos
                    Zc                 = -self.d / (self.a/self.fx*(imgpos[0]-self.cx) + self.b/self.fy*(imgpos[1] - self.cy) + self.c)
                    weldpoint2camera   = np.array([[Zc/self.fx*(imgpos[0]-self.cx)], [Zc/self.fy*(imgpos[1]-self.cy)], [Zc] , [1]])
                    tool2robot         = self.vis.homogeneous(NowPos[0])
                    weldpoint2robot    = tool2robot.dot(self.eye2hand).dot(weldpoint2camera).flatten()[0:3]
                    print(weldpoint2robot)
                    weldpoint2robot[0] = weldpoint2robot[0]
                    weldpoint2robot[1] = weldpoint2robot[1] - 15
                    weldpoint2robot[2] = weldpoint2robot[2] - 5
                    if weldpoint2robot[2] < -485:
                        weldpoint2robot[2] = -485
                    WeldPoint.append(np.round(weldpoint2robot,3))
                    # print("distance: ", np.linalg.norm(WeldPoint[0][0] - CurrentPos[0]))
                    if np.linalg.norm(WeldPoint[0][0] - CurrentPos[0]) < 10:
                        StartWelding = True
                        # print("start true")
                    # print("WeldPoint LEN: ", len(WeldPoint))
                    del NowPos[0]
                    del ImgArr[0]
                    img = cv.resize(img, (960, 520), interpolation = cv.INTER_AREA)
                    cv.imshow("SoftwareTask 2", img)
                    cv.waitKey(1)
            except:
                print("Software error: ", sys.exc_info())
            # finally:
                # print("leng:", len(NowPos), len(ImgArr))

ImgArr = []
StartWelding = False
WeldDone = False
NowPos = []
WeldPoint = []

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
software.start()