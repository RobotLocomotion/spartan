#!/usr/bin/env python

import numpy as np
import cv2
import glob
import argparse

def display_all_detections_matching_regex(image_file_or_dir_regex):
    # basically straight from https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_calib3d/py_calibration/py_calibration.html

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    chessboard_size_tuple = (7,6)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((chessboard_size_tuple[0]*chessboard_size_tuple[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboard_size_tuple[0],0:chessboard_size_tuple[1]].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    #"/home/peteflo/spartan/calibration_data/20180222-005312_ir/*.bmp"
    images = glob.glob(image_file_or_dir_regex)

    print images

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        print "Attempting to detect chessboard in img", fname

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size_tuple ,None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            print "True: detected chessboard, visualizing"
            objpoints.append(objp)

            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboard_size_tuple, corners,ret)
        else:
            print "False: (could not detect chessboard)"
        
        cv2.imshow('img',img)
        cv2.waitKey(200)
        name = raw_input("continue? press enter\n")
        

    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--images_regex", type=str, required=True, help="regular expression of image(s) to show detection")
    args = parser.parse_args()
    display_all_detections_matching_regex(args.images_regex)

