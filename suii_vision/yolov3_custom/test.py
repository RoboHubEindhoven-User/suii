from yolo import Yolo
import cv2
import time

if __name__ == "__main__":
    y = Yolo()
    y.load_model()

    dev = cv2.VideoCapture(1)
    while True:
        _, img = dev.read()
        y.run(img, True)
        #time.sleep(0.5)
