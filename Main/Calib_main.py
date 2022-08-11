'''
Main code for all the calibration processses.
    1. Camera calibration
    2. Laser calibration
    3. Robot hand-eye calibration

'''

from Calib_camera import monoCalibrate
from Calib_handeye import handeyeCalibrate
from Calib_laser import laserCalibrate
import time


process_start = time.time()

"""
Run calibration only ONCE right after setting up/modifying the system.
Calibration tasks need to be done everytime the laser vision system or welding torch 
is dissembled, changed in baseline distance, modified. 
"""

# Calibration for camera
monoCalibrate()

# Calibration for hand-eye robot
handeyeCalibrate()

# Calibration for laser triangulation 
laserCalibrate()

process_end = time.time()
print("\n\t* * * FULL PROCESS COMPLETE * * *")
print("\t:: Total process time: %.3f s" %(process_end-process_start))