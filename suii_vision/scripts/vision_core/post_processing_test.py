#!/usr/bin/env python
#
# Created by Jeroen Bongers in name of RoboHub Eindhoven
#

from post_processing_v3 import PostProcessing
import cv2
import pyrealsense2 as rs
import numpy as np


class PostProcessingTest:

    def __init__(self):
        self.test = PostProcessing()
        self.postprocessing_test() 

    def postprocessing_test(self):
            # Create a context object. This object owns the handles to all connected realsense devices
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)  
            # Create a pipeline object. This object configures the streaming camera and owns it's handle
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())       
            build_center = self.test.build_center("Bolt",(0,0,640,480),color_image,True)
            build_view = self.test.build_view()
            reset_view = self.test.reset_view()
            print(build_center)
            print(build_view)
            print(reset_view)
            cv2.waitKey(5000)
            pipeline.stop()

if __name__ == '__main__':
    PostProcessingTest()
