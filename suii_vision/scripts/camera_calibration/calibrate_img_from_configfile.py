#!/usr/bin/env python

#https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#goal

import numpy as np
import cv2
import glob
import yaml
import json

data = np.load('mtx.npz')
mtx = data['mtx']
print(mtx)

data = np.load('dist.npz')
dist = data['dist']
print(dist)

img = cv2.imread('calibration_31.png')
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

#print("mtx: {}, dist: {}, newcameramtx: {}".format(mtx,dist,newcameramtx))
#undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

#crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]

cv2.imshow("img",img)
cv2.imshow("dst",dst)
cv2.waitKey(10000)

cv2.imwrite('calibresult.png',dst)
img = cv2.imread('calibresult.png')

cv2.destroyAllWindows()