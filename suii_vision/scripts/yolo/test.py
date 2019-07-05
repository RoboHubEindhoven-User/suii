from yolo import Yolo
import cv2
import time
import pyrealsense2 as rs
import numpy
from processing import PostProcessing

if __name__ == "__main__":
    y = Yolo()
    y.load_model()
    dev = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    dev.start(config)
    p = PostProcessing()

    while True:
        mf = dev.wait_for_frames()
        rc = mf.get_color_frame()
        img_raw = numpy.asarray(rc.get_data())
        _, fi = p.filter_img(img_raw, 0, 0, 0, 0)
        variable = y.run(fi, True)

        #for x in variable:
        #    print(x[0])
        #    p.build_center(x[0], x[1], img_raw, True)
        #    dt = p.build_view()
        #    p.reset_view()
        #time.sleep(0.5) 