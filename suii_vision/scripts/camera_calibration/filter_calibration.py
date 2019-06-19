#!/usr/bin/env python
#tutorial link: https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/
#Tutorail link: http://opencvpython.blogspot.com/2012/06/contours-2-brotherhood.html

#import the necesarry packages
import argparse
import imutils
import cv2
import numpy as np 
import time
import math
from sensor_msgs.msg import Image
import rospy 
import pyrealsense2 as rs


#Threshold for drawing contours
MIN_THRESH = 50 #100
#Dimension test square to measure mm/pixel ratio
l_square = 40 #mm
w_square = 40 #mm
#calcutaing mm/pixel ratio from measurements:
l_per_pix = l_square/94.0 #in mm per pix
w_per_pix = w_square/94.0 #in mm per pix
#MidPoint in pixels
x_mid = 320 # in pixels
y_mid = 240 # in pixels
#create variables for square check
l_pix = 0
w_pix = 0 
#ROI
left_upper = (0,0) #x,y
right_lower = (640,480) #x,y
#setting up camera stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config) 

cX = 0 
cY = 0
rect_biggest = 0
center_biggest = 0
cX_biggest = 0
cY_biggest = 0

def nothing(w):
    pass

cv2.namedWindow("blurred")
cv2.createTrackbar("filter_val", "blurred", 0, 255, nothing)
cv2.createTrackbar("area_val", "blurred",0,30,nothing)

cv2.namedWindow("edges")
cv2.createTrackbar("lower_val", "edges", 0, 255, nothing)
cv2.createTrackbar("upper_val", "edges",0,255,nothing)

while True:
    # Create a pipeline object. This object configures the streaming camera and owns it's handle
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data()) 
    
    #frame = cv2.imread("material_1.png",cv2.IMREAD_COLOR)
    cv2.imshow("frame", color_image)
    #img_pub = rospy.Publisher('/img_pub', Image, queue_size=10)
    cv2.waitKey(3)
    # SPACE pressed
    filter_val = cv2.getTrackbarPos("filter_val","blurred")
    area_val = cv2.getTrackbarPos("area_val","blurred")
    lower_val = cv2.getTrackbarPos("lower_val","edges")
    upper_val = cv2.getTrackbarPos("upper_val","edges")
    
   
    img = color_image
    blurred = cv2.bilateralFilter(color_image,area_val,filter_val,filter_val)#9,75,75#9,50,50
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, lower_val, upper_val) #0,200
    cv2.imshow("blurred",blurred)
    #cv2.imshow("gray",gray)
    cv2.imshow("edges",edges)

    #draw midpoint circle
    cv2.circle(img, (x_mid, y_mid), 7, (0, 255, 0), -1)    
    # find contours in the thresholded image
    cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # loop over the contours
    for c in cnts:
        if cv2.contourArea(c) > MIN_THRESH:
            # compute the center of the contour
            M = cv2.moments(c)
            if(M["m00"] == 0): 
                M["m00"]=1
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            if(cX > left_upper[0] and cX < right_lower[0]  and cY > left_upper[1] and cY < right_lower[1]):
            #only apply for ROI    
                rect = cv2.minAreaRect(c)
                #print("square format: {}".format(rect))
                center = rect[0]
                cX = center[0]
                cY = center[1]
                #print("centerasdfafds{}".format(center))
                dimen = rect[1]
                l_pix_temp = dimen[0]
                w_pix_temp = dimen[1]
                #print("l pix {} ".format(l_pix_temp))
                #print("w pix {} ".format(w_pix_temp))
                #check for only the biggest square
                surface_temp = l_pix_temp * w_pix_temp
                surface = l_pix * w_pix
                if(surface_temp > surface):
                    l_pix = l_pix_temp
                    w_pix = w_pix_temp
                    rect_biggest = rect
                    cX_biggest = int(cX)
                    cY_biggest = int(cY)
                    #print("big_cx {} big_cy {}".format(cX_biggest, cY_biggest))
                    #print(rect_biggest)
                        

    #check if ther is a square detected
    if cX_biggest != 0:
        #print("Center: cX = {} cY = {}".format(cX,cY))
        # draw the contour and center of the shape on the image
        cv2.drawContours(img, [c], -1, (255, 0, 0), 1)
        cv2.circle(img, (cX_biggest, cY_biggest), 7, (255, 255, 255), -1)                   

        #print("l {} w {}".format(l_pix, w_pix))
        #printin real lenght of object
        real_length = l_pix * l_per_pix
        real_width = w_pix * w_per_pix
        #print("Object lenght = {} mm, Object width = {} mm".format(real_length,real_width))
        #calculate coordinate from midpoint
        x_orientation = cX_biggest - x_mid  #in pixels
        y_orientation = y_mid - cY_biggest#in pixels
        x_orientation = x_orientation*l_per_pix
        y_orientation = y_orientation*w_per_pix
        #print("X orientation = {} mm, Y orientation = {} mm".format(x_orientation,y_orientation))
        
        degrees = rect_biggest[2]
        radians = ((degrees*math.pi)/180)
        #print("Theta in radians: {}.".format(radians))
        box = cv2.boxPoints(rect_biggest)
        box = np.int0(box)
        cv2.drawContours(img,[box],0,(0,0,255),2)
        cv2.rectangle(img, left_upper, right_lower, (125,0,125), 2)
        cv2.imshow("img",img)
        #drawing line and line midpoint
        #cv2.line(img,(self.cols-1,self.righty),(0,self.lefty),(0,255,0),2)
        #cv2.circle(img, (self.x, self.y), 7, (0, 255, 0), -1) 
        #img_pub.publish(img, "bgr8")
        l_pix = 0
        w_pix = 0
        rect_biggest = 0
        cX_biggest = 0
        cY_biggest = 0
  
    time.sleep(0.02)

pipeline.stop()      
cv2.destroyAllWindows()        
cv2.waitKey(0)
