import urllib
import cv2
import numpy as np
import time
import os

# Replace the URL with your own IPwebcam shot.jpg IP:port
url='http://145.93.60.207:8080/shot.jpg'

img_counter = 0

while True:

    # Use urllib to get the image and convert into a cv2 usable format
    imgResp=urllib.urlopen(url)
    imgNp=np.array(bytearray(imgResp.read()),dtype=np.uint8)
    img=cv2.imdecode(imgNp,-1)

    # put the image on screen
    cv2.imshow('IPWebcam',img)
    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break

    elif k%256 == 32:
        # SPACE pressed
        # Save image with img_name at folder specified in path, and print verification
        img_name = "opencv_frame_{}.png".format(img_counter)
        path = '/home/ros/workspace/datasets/darknet_v3_competition_objects/all_images'
        cv2.imwrite(os.path.join(path , img_name), img)
        print("{} written!".format(img_name))
        img_counter += 1

cam.release()

cv2.destroyAllWindows()