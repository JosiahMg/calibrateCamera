#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 16 08:45:25 2019

@author: hmeng
"""

import numpy as np
import cv2

objp_dict = {
	1: (9, 5),
	2: (9, 6),
	3: (9, 6),
	4: (9, 6),
	5: (9, 6),
	6: (9, 6),
	7: (9, 6),
	8: (9, 6),
	9: (9, 6),
	10: (9, 6),
	11: (9, 6),
	12: (9, 6),
	13: (9, 6),
	14: (9, 6),
	15: (9, 6),
	16: (9, 6),
	18: (9, 6),
	17: (9, 6),
	19: (9, 6),
	20: (9, 6),
}

objp_list = []
corners_list = []

for k in objp_dict:
    nx, ny = objp_dict[k]
	# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((nx*ny,3), np.float32)
    
    objp[:,:2] = np.mgrid[0:nx, 0:ny].T.reshape(-1,2)
	# Make a list of calibration images
    fname = 'camera_cal/calibration%s.jpg' % str(k)
    img = cv2.imread(fname)

	# Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	# Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

	# If found, save & draw corners
    if ret == True:
        # Save object points and corresponding corners
        objp_list.append(objp)
        corners_list.append(corners)
		# Draw and display the corners
		#cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
		#plt.imshow(img)
		#plt.show()
		#print('Found corners for %s' % fname)
    else:
        print('Warning: ret = %s for %s' % (ret, fname))
            
img = cv2.imread('camera_cal/calibration1.jpg')
img_size = (img.shape[1], img.shape[0])
'''
mtx : 
dist:


'''
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objp_list, corners_list, img_size,None,None)


dst = cv2.undistort(img, mtx, dist, None, mtx)
com_img = np.hstack((img, dst))
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.imshow('image', com_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

    
    
