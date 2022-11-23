# import numpy as np
# from matplotlib import pyplot as plt

# from skimage.measure import LineModelND, ransac

# rng = np.random.default_rng()

# # generate coordinates of line
# x = np.arange(-200, 200)
# y = 0.2 * x + 20
# data = np.column_stack([x, y])

# print(data)

with open('D:/Workspace/Code/Welding_Robot/3D_Laser_Vision_Welding_Robot/MyCamera/Scan_data/Scan_pos.txt', 'r') as f:
    l = [[float(num) for num in line.split('\t')] for line in f]
print(l)