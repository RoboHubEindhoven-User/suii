# First import the library
import pyrealsense2 as rs
import cv2
import numpy as np

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

cv2.imshow("test_img",color_image)

cv2.waitKey(1000)

    

pipeline.stop()