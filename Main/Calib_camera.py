'''
Code for camera calibration fnction
    1. Load the camera images
    2. Extract the chessboard corners
    3. Calibrate the camera
    4. Save the camera matrix and the distortion coefficients
'''

from MyLib import save_coefficients
from GlobalVariables import *
import cv2 as cv
import numpy as np
import glob
import time

# termination criteria 
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
calibrate_criteria = (cv.TermCriteria_COUNT + cv.TermCriteria_EPS, 500, 0.0001)
find_chessboard_flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FILTER_QUADS + cv.CALIB_CB_NORMALIZE_IMAGE

def calibrate(dirpath, prefix, image_format, square_size, width, height):
    """ Apply camera calibration operation for images in the given directory path. """
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    objp = objp * square_size  # Create real world coords. Use your metric.

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space.
    imgpoints = []  # 2d points in image plane.

    # Directory path correction. Remove the last character if it is '/'
    if dirpath[-1:] == '/':
        dirpath = dirpath[:-1]

    # Get the images
    images = glob.glob(dirpath + '/' + prefix + '*.' + image_format)

    # Iterate through the pairs and find chessboard corners. Add them to arrays
    # If openCV can't find the corners in an image, we discard the image.
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (width, height), None, find_chessboard_flags)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (15, 15), (-1, -1), criteria)
            imgpoints.append(corners2)
            # # Draw and display the corners
            # img = cv.drawChessboardCorners(img, (width, height), corners2, ret)
            # cv.imshow('img',cv.resize(img,(720,480)))
            # cv.waitKey(20)
        else:
            print("CHECKERBOARD NOT DETECTED!\t---> IMAGE PAIR: ", fname)
    calib_flags = (cv.CALIB_FIX_PRINCIPAL_POINT + cv.CALIB_FIX_ASPECT_RATIO + cv.CALIB_ZERO_TANGENT_DIST + cv.CALIB_RATIONAL_MODEL + cv.CALIB_FIX_K3 + cv.CALIB_FIX_K4 + cv.CALIB_FIX_K5)

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, cv.CALIB_USE_INTRINSIC_GUESS, criteria=calibrate_criteria)
    return [ret, mtx, dist, rvecs, tvecs]

def monoCalibrate():
    # Preliminary information of checkerboard
    width = 10
    height = 7
    square_size = 15

    # Information of saving and requesting data path
    save_camera_params_path = camera_params
    checker_path = checkerboard_calib_cam_path
    prefix = 'checkerboard_'

    print("\n MONO CALIBRATING _______________________________")
    start = time.time()
    ret, mtx, dist, rvecs, tvecs = calibrate(checker_path, prefix, img_format, square_size, width, height)
    end1 = time.time()
    save_coefficients(mtx, dist, save_camera_params_path)
    print(f"|\t:: LEFT RMS:\t{ret}\t|")
    print('|\t:: Time consumed: %.2lf s\t\t|' %(end1-start))
    print("|________________________ MONO CALIBRATION DONE |")
