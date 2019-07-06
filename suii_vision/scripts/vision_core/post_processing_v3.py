#!/usr/bin/env python
#
# Created by Jeroen Bongers in name of RoboHub Eindhoven
#
# 

import cv2
import imutils
import numpy as np
import math

class PostProcessing:

    def __init__(self):
        """[Defining global variables]
        """     
        self.left_upper = []
        self.right_lower = []
        # Dimension test square to measure mm/pixel ratio
        l_square = 40 #mm
        w_square = 40 #mm
        # Calcutaing mm/pixel ratio from measurements from 40 cm hight:
        self.l_per_pix = l_square/58.0 # In mm per pix
        self.w_per_pix = w_square/58.0  # In mm per pix
        self.cx_biggest = None
        self.object_list = []
        self.debug = False
        # Camera calibration files
        data = np.load('/home/suii/catkin_ws/src/suii/suii_vision/scripts/camera_calibration/mtx.npz')
        self.mtx = data['mtx']
        data = np.load('/home/suii/catkin_ws/src/suii/suii_vision/scripts/camera_calibration/dist.npz')
        self.dist = data['dist'] 
        self.mal_or_object = "object"

    def build_center(self, object_name, roi, img, debug):
        """[Searches for requested object_name and retruns a True when found, a False when not found. If found the x ,y, theta are added to object_list]
        
        Arguments:
            object_name {[str]} -- [Name of required object]
            roi {[tuple]} -- [Gives values for left upper and right lower corner]
            img {[numpy.ndarray]} -- [Input image]
            debug {[bool]} -- [If true than the code displays result image]
        """
        self.left_upper = []
        self.right_lower = []
        self.cx_biggest = None
        name_var = object_name
        self.debug = debug
        self.left_upper = roi[0:2]
        self.right_lower = roi[2:4]
        # Get filter  val
        area_val, blur_val, lower_val, upper_val = self.get_filter_vals(name_var)
        # Filters the image
        img_edges, img = self.filter_img(img, area_val, blur_val, lower_val, upper_val)
        # Get contours
        if self.mal_or_object == "object":
            contours = self.get_contours(img_edges)
            # Get biggest possible rectangle for getting outer contour
            self.get_biggest_rect(contours,img_edges)
            #calcutale and draw the x,y,theta
            tf_vals = self.calculate_draw_result(img)
        elif self.mal_or_object == "mal":
            tf_vals = self.roi_center_rotation(img)
        #add values to list
        list_vars = [name_var,tf_vals[0],tf_vals[1],tf_vals[2]]
        if self.debug:
            print("name_var: {}, x {}, y {}, z {}".format(name_var,tf_vals[0],tf_vals[1],tf_vals[2]))
            cv2.imshow('img_var',img)
            cv2.waitKey(5000)
            cv2.destroyAllWindows()
        if tf_vals[3] == True:
            self.object_list.append(list_vars)
            return True  
        else: 
            return False
        
        
    def build_view(self):
        """[Returning object_list]
        
        Returns:
            [list] -- [List contain object names and their x, y, theta]
        """
        return self.object_list

    def reset_view(self):
        """[Clears object_list]
        
        Returns:
            [bool] -- [Returns true when object_list is cleared]
        """
        self.object_list = []
        return True

    def get_filter_vals(self, obj_name):
        """[Gets the filter values for requested objects]
        
        Arguments:
            obj_name {[str]} -- [Object name]
        
        Returns:
            [int] -- [Returns four integers for changing filter settings]
        """        
        black_list = ["F20_20_B","S40_40_B","M20_100","R20","Bearing","Motor","Bearing"]
        alu_list = ["F20_20_G","S40_40_G"]
        nut_list = ["M20", "M30"]
        shiny_list = ["Bearing_Box","Axis","Distance_Tube"] 
        basket_list = ["Blue_Container_Top","Blue_Container_Front","Red_Container_Top","Red_Container_Front"]
        malle_list = ["F20_20_Mall","F40_40_Mall","R20_Mall","Bolt_Mall","M20_Mall","M30_Mall"]
        self.mal_or_object = "object"
        if obj_name in black_list:
            area_val = 6
            blur_val = 126
            lower_val = 75
            upper_val = 251
        elif obj_name in nut_list:
            area_val = 3
            blur_val = 149
            lower_val = 48
            upper_val = 103
        elif obj_name in alu_list:
            area_val = 6
            blur_val = 30
            lower_val = 55
            upper_val = 138
        elif obj_name in shiny_list:
            area_val = 0
            blur_val = 0
            lower_val = 20
            upper_val = 51
        elif obj_name in basket_list:
            area_val = 1
            blur_val = 44
            lower_val = 27
            upper_val = 99
        elif obj_name in malle_list:
            self.mal_or_object = "mal"
            area_val = 1
            blur_val = 44
            lower_val = 27
            upper_val = 99
        else:
            area_val = 6
            blur_val = 126
            lower_val = 75
            upper_val = 251

        if self.debug:
            print("area {} blur {} lower {} upper {}".format(area_val,blur_val,lower_val,upper_val))
        return area_val, blur_val, lower_val, upper_val
            
    def filter_img(self,img, area_val, blur_val, lower_val, upper_val):
        """[Gets all the edges on the image]
        
        Arguments:
            img {[numpy.ndarray]} -- [Input image]
            area_val {[int]} -- [Pixel area value]
            blur_val {[int]} -- [Blurring threshhold value]
            lower_val {[int]} -- [Lower threshhold value]
            upper_val {[int]} -- [Upper threshhold value]
        
        Returns:
            [numpy.ndarray] -- [Output matrix from canny edge detection]
        """        
        image = img
        h,  w = image.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),1,(w,h))
        dst = cv2.undistort(image, self.mtx, self.dist, None, newcameramtx)
        x,y,w,h = roi
        image = dst[y:y+h, x:x+w]
        blurred = cv2.bilateralFilter(image ,area_val,blur_val,blur_val)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, lower_val, upper_val)
        
        return edges, image
    
    def get_contours(self,edges):
        """[Get the contour from the canny edges]
        
        Arguments:
            edges {[numpy.ndarray]} -- [Output matrix from canny edge detection]
        
        Returns:
            [list] -- [Retruns the contours of image]
        """
        img_edges = edges
        cnts = cv2.findContours(img_edges.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        return cnts

    def get_biggest_rect(self,contours, img_edges):
        """[Gets the smalles possible rectangle fitting the object for orientation and drawing a line to calcualte rotation.]
        
        Arguments:
            contours {[list]} -- [Matrix which contains the image contours]
            img_edges {[numpy.ndarray]} -- [Output matrix from canny edge detection]
        """        
        l_pix = 0
        w_pix = 0
        MIN_THRESH = 50
        for c in contours:
            if cv2.contourArea(c) > MIN_THRESH:

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
                    surface = l_pix * w_pix
                    
                    if(surface_temp > surface):
                        self.rect_biggest = rect
                        center = rect [0]
                        self.biggest_c = c
                        self.cx_biggest = center[0]
                        self.cy_biggest = center[1]
                        self.l_pix_biggest = l_pix_temp
                        self.w_pix_biggest = w_pix_temp
                        #calculate line values for rotation
                        self.rows,self.cols = img_edges.shape[:2]
                        [vx,vy,self.x,self.y] = cv2.fitLine(self.biggest_c, cv2.DIST_L2,0,0.01,0.01)
                        self.lefty = int((-self.x*vy/vx) + self.y)
                        self.righty = int(((self.cols-self.x)*vy/vx)+self.y)
                        self.top = self.cols-1,self.righty
                    
    
    def calculate_draw_result(self,img):
        """[Calculates the orientation and rotation results and draws the results on the image]
        
        Arguments:
            img {[numpy.ndarray]} -- [Input image]
        
        Returns:
            [list] -- [List includes the oriantation, rotation and a true bool or a false and 0's]
        """        
        #MidPoint of camera in pixels
        x_mid = 320 # in pixels
        y_mid = 240 # in pixels
        if self.cx_biggest is not None:
            cv2.circle(img, (x_mid, y_mid), 7, (255, 255, 255), -1) 
            # draw the contour of the shape on the image
            cv2.drawContours(img , [self.biggest_c], -1, (255, 0, 0), 2)
            real_length = self.l_pix_biggest * self.l_per_pix
            real_width = self.w_pix_biggest * self.w_per_pix
            if self.debug:
                print("pix length: {}, pix width: {}".format(self.l_pix_biggest,self.w_pix_biggest))
                print("Object lenght: {} mm, Object width: {} mm".format(real_length,real_width))
            #calculate coordinate from midpoint
            x_orientation = self.cx_biggest - x_mid  #in pixels (from line)
            y_orientation = y_mid - self.cy_biggest #in pixels (from line)
            x_orientation = (x_orientation*self.l_per_pix)/1000 #calculate pixel to meter and invert
            y_orientation = (y_orientation*self.w_per_pix)/1000 #calculate pixel to meter and invert
            draw_x = int(self.cx_biggest)
            draw_y = int(self.cy_biggest)        
            cv2.circle(img , (draw_x,draw_y), 7, (125, 0, 125), -1)
            box = cv2.boxPoints(self.rect_biggest)
            box = np.int0(box)
            cv2.drawContours(img ,[box],0,(0,0,255),2)
            #get rotation of line in radians
            myradians = math.atan2(self.top[1]-self.cy_biggest, self.top[0]-self.cx_biggest)
            #drawing rectangle ROI
            cv2.rectangle(img , self.left_upper, self.right_lower, (125,0,125), 2)
            #drawing line and line midpoint
            cv2.line(img ,(self.cols-1,self.righty),(0,self.lefty),(0,255,0),2)
            cv2.circle(img , (self.x, self.y), 7, (0, 255, 0), -1)
            return x_orientation,y_orientation,myradians,True 
        else:
            x_orientation,y_orientation,myradians,boolean = self.roi_center_rotation(img)
            return x_orientation,y_orientation,myradians,boolean

    def roi_center_rotation(self,img):
        #MidPoint of camera in pixels
        x_mid = 320 # in pixels
        y_mid = 240 # in pixels
        cv2.circle(img, (x_mid, y_mid), 7, (255, 255, 255), -1) 
        #rotation
        width = self.right_lower[0] - self.left_upper[0]
        lenght = self.right_lower[1] - self.left_upper[1]
        if width > lenght:
            myradians = 0.001
        else:
            myradians = -(0.5*math.pi)
        #center
        x_mid_roi = (width/2) + self.left_upper[0]
        y_mid_roi = (lenght/2) + self.left_upper[1]
        draw_x = int(x_mid_roi)
        draw_y = int(y_mid_roi)
        x_orientation = x_mid_roi - x_mid  #in pixels (from line)
        y_orientation = y_mid - y_mid_roi #in pixels (from line)
        x_orientation = (x_orientation*self.l_per_pix)/1000 #calculate pixel to meter and invert
        y_orientation = (y_orientation*self.w_per_pix)/1000 #calculate pixel to meter and invert   
        cv2.circle(img , (draw_x,draw_y), 7, (125, 0, 125), -1)
        #drawing rectangle ROI
        cv2.rectangle(img , self.left_upper, self.right_lower, (125,0,125), 2)
        return x_orientation,y_orientation,myradians,True 
            
