#!/usr/bin/env python
#
# Created by Jeroen Bongers in name of RoboHub Eindhoven
#
# 


#Questions:
#what about running programm. Now every time runned it will shut down and lose list data

import cv2
import imutils
import numpy as np
import math

class PostProcessing:

    def __init__(self):
        '''defining global variables'''       
        self.name_var = ""
        self.roi_var = []
        self.img_edges = None
        self.tf_array = []
        self.MIN_THRESH = 10
        self.left_upper = []
        self.right_lower = []
        tf_vals = []
        self.l_pix = 0
        self.w_pix = 0
        #MidPoint of camera in pixels
        self.x_mid = 320 # in pixels
        self.y_mid = 240 # in pixels
        #Dimension test square to measure mm/pixel ratio
        l_square = 40 #mm
        w_square = 40 #mm
        #calcutaing mm/pixel ratio from measurements from 40 cm hight:
        self.l_per_pix = l_square/84.4927978516  #in mm per pix
        self.w_per_pix = w_square/84.6966323853  #in mm per pix
        self.cx_biggest = None
        self.object_list = []
        self.list_vars = []

    def build_center(self, object_name, roi, img):
        '''recieves object_name, roi, img => add TF name to TF array and returns True if succeeded'''
        self.name_var = object_name
        self.left_upper = roi[0:2]
        self.right_lower = roi[2:4]
        self.img_var = img
        self.img_edges = self.filter_img(self.img_var)
        #get contours
        contours = self.get_contours(self.img_edges)
        #get biggest possible rectangle for getting outer contour
        self.get_biggest_rect(contours)
        #calcutale and draw the x,y,theta
        tf_vals = self.calculate_draw_result(self.img_var)
        #add values to list
        self.list_vars = [self.name_var,tf_vals[0],tf_vals[1],tf_vals[2]]
        if tf_vals[3] == True:
            self.object_list.append(self.list_vars)
            return(True)  
        else: 
            return(False)
        #DELETE IMSHOWS, only used for testing code!!!!!
        #cv2.imshow('img_var',self.img_var)
        
    def build_view(self):
        '''returns TF array'''
        return(self.object_list)

    def reset_view(self):
        '''reset array, return true if succeeded'''
        self.object_list = []
        return(True)

    def filter_img(self,img):
        image = img
        blurred = cv2.bilateralFilter(image ,6,75,75)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 5, 40)
        return(edges)
    
    def get_contours(self,edges):
        img_edges = edges
        cnts = cv2.findContours(img_edges.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        return(cnts)

    def get_biggest_rect(self,contours):
        for c in contours:
            if cv2.contourArea(c) > self.MIN_THRESH:
                #get center of contour
                M = cv2.moments(c)
                if(M["m00"] == 0): 
                    M["m00"]=1
                cx_temp = int(M["m10"] / M["m00"])
                cy_temp = int(M["m01"] / M["m00"])
                #check if center of contour is in ROI
                if(cx_temp > self.left_upper[0] and cx_temp < self.right_lower[0]  and cy_temp > self.left_upper[1] and cy_temp < self.right_lower[1]):
                    #biggest rectangle around object for orientation 
                    rect = cv2.minAreaRect(c)
                    dimen = rect[1]
                    l_pix_temp = dimen[0]
                    w_pix_temp = dimen[1]
                    surface_temp = l_pix_temp * w_pix_temp
                    surface = self.l_pix * self.w_pix
                    if(surface_temp > surface):
                        self.rect_biggest = rect
                        center = rect [0]
                        self.biggest_c = c
                        self.cx_biggest = center[0]
                        self.cy_biggest = center[1]
                        self.l_pix_biggest = l_pix_temp
                        self.w_pix_biggest = w_pix_temp
                        #calculate line values for rotation
                        self.rows,self.cols = self.img_edges.shape[:2]
                        [vx,vy,self.x,self.y] = cv2.fitLine(self.biggest_c, cv2.DIST_L2,0,0.01,0.01)
                        self.lefty = int((-self.x*vy/vx) + self.y)
                        self.righty = int(((self.cols-self.x)*vy/vx)+self.y)
                        self.top = self.cols-1,self.righty 
        #print("cx{}, cy{}, l_pix{}, w_pix{}".format(self.cx_biggest,self.cy_biggest,self.l_pix_biggest, self.w_pix_biggest))           
    
    def calculate_draw_result(self,img):
        if self.cx_biggest is not None:
            #delete print lines later
            print("pix length: {}, pix width: {}".format(self.l_pix_biggest,self.w_pix_biggest))
            cv2.circle(img, (self.x_mid, self.y_mid), 7, (255, 255, 255), -1) 
            # draw the contour of the shape on the image
            cv2.drawContours(self.img_var , [self.biggest_c], -1, (255, 0, 0), 2)
            #printin real lenght of object for testing purpose
            real_length = self.l_pix * self.l_per_pix
            real_width = self.w_pix * self.w_per_pix
            #print("Object lenght: {} mm, Object width: {} mm".format(real_length,real_width))
            #calculate coordinate from midpoint
            x_orientation = self.cx_biggest - self.x_mid  #in pixels (from line)
            y_orientation = self.y_mid - self.cy_biggest #in pixels (from line)
            x_orientation = x_orientation*self.l_per_pix
            y_orientation = y_orientation*self.w_per_pix
            draw_x = int(self.cx_biggest)
            draw_y = int(self.cy_biggest)        
            #print("X orientation: {} mm, Y orientation: {} mm".format(x_orientation,y_orientation))
            cv2.circle(self.img_var , (draw_x,draw_y), 7, (125, 0, 125), -1)
            box = cv2.boxPoints(self.rect_biggest)
            box = np.int0(box)
            cv2.drawContours(self.img_var ,[box],0,(0,0,255),2)
            #get rotation of line in radians
            myradians = math.atan2(self.top[1]-self.cy_biggest, self.top[0]-self.cx_biggest)
            #print("Object rotation: {} RAD".format(myradians))
            #print("----------------------------------------------")
            #drawing rectangle ROI
            cv2.rectangle(self.img_var , self.left_upper, self.right_lower, (125,0,125), 2)
            #drawing line and line midpoint
            cv2.line(self.img_var ,(self.cols-1,self.righty),(0,self.lefty),(0,255,0),2)
            cv2.circle(self.img_var , (self.x, self.y), 7, (0, 255, 0), -1)
            return(x_orientation,y_orientation,myradians,True)
        else:
            return(0,0,0,False)


