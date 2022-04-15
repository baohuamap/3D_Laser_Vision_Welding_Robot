import threading
import time

class RobotTask(threading.Thread):
   def __init__(self):
      threading.Thread.__init__(self)
   def run(self):
      global event
      while True:
         try:
            if event:
               print("event")
               event = False
         except:
            pass


class VisionTask(threading.Thread):
   def __init__(self):
      threading.Thread.__init__(self)
   def run(self):
      print("start")
      global event
      while True:
         try:
            time.sleep(0.5)
            print("event end")
            event = True
         except:
            pass

t = time.time()
# Create new threads
ImgArr = []
event = False
thread1 = RobotTask()
thread2 = VisionTask()

# Start new Threads
thread1.start()
thread2.start()

thread1.join()
thread2.join()
print("total time: ", time.time() - t)