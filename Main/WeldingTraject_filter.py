from matplotlib import pyplot as plt
from GlobalVariables import *
import numpy as np
from MyLib import savematrix


with open(welding_trajectory, 'r') as f:
    positions = [[float(num) for num in line.split('\t')] for line in f]

positions = np.array(positions)

# https://stackoverflow.com/questions/2828059/sorting-arrays-in-numpy-by-column 
positions[positions[:, 0].argsort()] #sort by column 1

# positions.sort(key=natural_keys)

print(positions)
savematrix(welding_trajectory_filtered, positions)