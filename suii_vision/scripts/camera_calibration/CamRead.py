import numpy as np
import cv2
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config) 
img_counter = 0
data = np.load('/home/jeroen/workspaces/image_processing/camera_calibration/mtx.npz')
mtx = data['mtx']
data = np.load('/home/jeroen/workspaces/image_processing/camera_calibration/dist.npz')
dist = data['dist'] 
 
while(True):
    # Create a pipeline object. This object configures the streaming camera and owns it's handle
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data()) 
 
    k = cv2.waitKey(1)
    cv2.imshow('Webcam', color_image)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break

    elif k%256 == 32:
        # SPACE pressed
        image = color_image
        h,  w = image.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        dst = cv2.undistort(image, mtx, dist, None, newcameramtx)
        x,y,w,h = roi
        image = dst[y:y+h, x:x+w]
        img_name = "calibration_{}.png".format(img_counter)
        cv2.imwrite(img_name, image)
        print("{} written!".format(img_name))
        img_counter += 1

pipeline.stop()

cv2.destroyAllWindows()