from matplotlib import pyplot as plt
from GlobalVariables import *
import numpy as np

robot_weldseam = 'D:/Workspace/Code/Welding_Robot/3D_Laser_Vision_Welding_Robot/RobotTrajectory/welding_traj_robot.txt'

with open(welding_trajectory_filtered, 'r') as f:
# with open(welding_trajectory, 'r') as f:
    positions = [[round(float(num),3) for num in line.split('\t')] for line in f]

with open(robot_weldseam, 'r') as f:
# with open(welding_trajectory, 'r') as f:
    robot_positions = [[round(float(num),3) for num in line.split('\t')] for line in f]

positions = np.array(positions)
robot_positions = np.array(robot_positions)
X = positions[:,0]
Y = positions[:,1]
Z = positions[:,2]

X_ROBOT = robot_positions[:,0]
Y_ROBOT = robot_positions[:,1]
Z_ROBOT = robot_positions[:,2]

'''
print ('X ', X)
print ('Y ', Y)
fig, ax = plt.subplots()
ax.plot(X, Y, '.b', alpha=0.6, label='weldseam')
# ax.plot(positions[outliers, 0], positions[outliers, 1], '.r', alpha=0.6,
#         label='Outlier data')
# ax.plot(line_x, line_y, '-k', label='Line model from all data')
# ax.plot(line_x, line_y_robust, '-b', label='Robust line model')
# ax.legend(loc='lower left')
plt.show()
'''

# --- Plot 3D --- 
fig = plt.figure()
ax  = fig.add_subplot(111, projection="3d")
# ax.scatter(X, Y, Z, c='r' , marker = '.')
# ax.scatter(X_ROBOT, Y_ROBOT, Z_ROBOT, c='g' , marker = 'x', linestyle = '-')
ax.plot(X, Y, Z, c='r', marker = '.', linestyle = '-')
ax.plot(X_ROBOT, Y_ROBOT,  Z_ROBOT, marker = 'x', linestyle = 'dashed')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
