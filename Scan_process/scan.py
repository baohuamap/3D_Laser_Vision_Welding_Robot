import time


scan_start = []
scan_stop = []

def homing():
    pass

def pos2Movjcommand(pos):
    return pos

def MovJ(pos):
    pass

def MovL(pos):
    pass

def RposC(pos):
    pass

def ScanProcess():
    '''
    1. move robot to home position
    2. move robot to scan the workpiece
    3. Apply RANSAC to find the weldseam_center
    4. Save the weldseam_center to buffer
    5. Save to txt file to be used    
    '''
    
    homing()
    time.sleep(5)
    start_point = pos2Movjcommand(scan_start)
    MovJ(start_point)
    
    # robot is moving to scan position
    # start capture
    # robot is scanning position
    # calculate and save the position

    pass

def main():
    ScanProcess()

main()