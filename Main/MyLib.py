'''
Library for 3D Laser Vision Welding Robot
    Vision and Robot Control modules

Author: Bao Hua
'''

import numpy as np
import cv2 as cv
import sys


def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

def savematrix(filename:str, matrix):
    with open(filename, 'w') as f:
        f.writelines('\t'.join(str(j) for j in i) + '\n' for i in matrix)