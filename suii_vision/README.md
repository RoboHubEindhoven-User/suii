# Suii_vision

This is a repository contains the vision solution for object detection for the Robocup@work competition that RoboHubEindhoven participates in. The vision works as followed. First the system will be determine a region of interest (ROI) around an object. This is a square drawn around the object. The ROI will be classified with a trained name. This is all done by using YOLO. The Second step is to determine the orientation and rotation. Image processing will take care of that! As a result you will get something like the following picture: 

![](https://github.com/RoboHubEindhoven/suii/blob/master/suii_vision/scripts/images/WhatsApp%20Image%202019-06-07%20at%2011.28.57%20AM.jpeg) 

We will devide this solution in to the two systems, YOLO Object detection and image processing. In this README we will also split up these parts. To learn more about our vision solution, you can read the links to the wiki's below. In this README there is a short summary of the results and a quickstart guide. In the wiki links you can read the full how-to-guides and more in depth information.

**YOLO:**
* [How does the custom YOLO python object detection work?](https://github.com/RoboHubEindhoven/suii/wiki/How-does-the-custom-YOLO-python-object-detection-work%3F/_edit)
* [How to train YOLO object detection](https://github.com/RoboHubEindhoven/suii/wiki/How-to-train-YOLO-object-detection)
* [How to use YOLO object detection](https://github.com/RoboHubEindhoven/suii/wiki/How-to-use-YOLO-object-detection)
* [YOLO object detection](https://github.com/RoboHubEindhoven/suii/wiki/YOLO-Object-detection)

**Image processing:**
* [Image_processing explained](https://github.com/RoboHubEindhoven/suii/wiki/Image_processing-explained)
* [Camera_calibration explained](https://github.com/RoboHubEindhoven/suii/wiki/Camera_calibration-explained)
* [Camera_accuracy_test results](https://github.com/RoboHubEindhoven/suii/wiki/Camera_accuracy_test-results)

# YOLO Object detection 

This repository consists of the [ultralytics python yolov3](https://github.com/ultralytics/yolov3) with some custom adjustments. If you want to learn more of how the YOLO object detection works and for how to use guides, please check out our [vision wiki](https://github.com/RoboHubEindhoven/suii/wiki/YOLO-Object-detection). At the end of the page where you land on you can find the links to all the guides you need to use the YOLO object detection.

**Below are some of the results on a custom trained database with the Robocup@work objects:**

![1](https://user-images.githubusercontent.com/39261806/58866927-d4df8180-86b9-11e9-82fa-778f1cc614f9.jpeg)

![index](https://user-images.githubusercontent.com/39261806/58866982-ede83280-86b9-11e9-80dd-680a19e43adf.jpeg)

## Getting started

For this project ubuntu 16.04 LTS is used with [nVidia CUDA toolkit 10.1](https://developer.nvidia.com/cuda-downloads) (For this you need an nVidia graphics card in the desktop we have, an gtx 1080 TI is used)

## Prerequisites

This package runs on python3.7 or later with the following packages:

* `numpy`
* `torch >= 1.1.0`
* `opencv-python`
* `tqdm`
* `matplotlib`
* `pycocotools`

To install the packages all in once run the following command in the `yolov3_custom` folder:

```bash
pip3 install -U -r requirements.txt
```

For cloning [ultralytics python yolov3](https://github.com/ultralytics/yolov3) use git clone into your own custom workspace:

```bash
git clone https://github.com/ultralytics/yolov3.git
```

## Usage

### Execute

To run the program execute the following command in a terminal:

```bash
python3 test.py
```

**note**: You might need to change the `cv2.VideoCapture(0)` to your desired camera input in the `test.py` file.

### Change file paths

To change the Config file which is used change the filepath in `defaults_dicts` in the `yolo.py` file to the `.cfg` file that you want to use. This is the same for the `.data` and `.weights` files.
Furthermore you can change the Confidence threshold `(conf_thresh)` value and the Non-maximum supression threshold `(nms_thresh)`.

**Config file**

The config file is needed to run the YOLO detection. For YOLOv3 there are 3 different versions;

* yolov3	(this is the full YOLO version which needs the most power to run, in theory it is the slowest but the most accurate but that depends on which graphics card you are using)
* yolov3-spp	(this is the middle version of YOLO which balances inbetween speed and accuracy)
* yolov3-tiny	(this is the smalles version of YOLO, which is the most inaccurate but can run on almost every graphics card)

**Data file**

This is a file containing the filepaths to all the necessary folders, for example the amount of `classes`, the filepath to the `train` folder where all the pictures are stored that you wish to train with, the filepath to the `valid` folder where all the pictures are that contain the objects but you do not want to train with, the filepath to the `.names` file where all the names of the classes are defined and the filepath to the `backup` folder where the training `.weight` files are stored.

**Weights file**

This is the file where the weights are stored from the custom dataset that you trained. This is needed for the detector to detect the trained objects.

### Troubleshoot

It might occure that the error occures that it cannot find opencv installed for python3, this might be due to ROS or other installations. When this happens run in the yolov3_custom folder the following command and retry after:

```bash
source envpy3
```

**Note:** You will have to change the filepath to the correct folder in the envpy3 code!

### Pretrained Weights
Below are the links to the pretrained weights:

* Darknet `*.weights` format: https://pjreddie.com/media/files/yolov3.weights
* PyTorch `*.pt` format: https://drive.google.com/drive/folders/1uxgUBemJVw9wZsdpboYbzUN4bcRhsuAI


# IMAGE_PROCESSING

This part of the repository contains multiple python scripts to process images and calibrate a camera.The processed image returns the x, y and theta of the center point of an object. This centerpoint is measured from the centerpoint of the camera. The x, y and theta will be used to create a TF, so a robotic arm can pick and place the object.

If you want to learn more of how the vision systems work, please check out our [wiki](https://github.com/RoboHubEindhoven/suii/wiki).

## Getting Started

This package only contains python2 scripts and make use of OpenCV-3.3.1-dev and numpy 1.11.0. Furthermore we used the BlasterX Senz3D camera. For this we need to have the librealsense2 drivers installed.

## Prerequisites

Assuming python2 or python3 and numpy are already installed, you only need to install OpenCV. If using python3, replace the "python2" to "python3". You can install OpenCV by running the following lines:

```
$ sudo apt-get install python-opencv
$ python2
import cv2 as cv
print(cv.__version__)
```
The last line will result in printing your installed OpenCV version. For more information about installin OpenCV, check the [OpenCV tutorial](https://docs.opencv.org/3.4/d2/de6/tutorial_py_setup_in_ubuntu.html)

Installing the librealsense2 drivers:

```
Sudo apt-get update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev
pip3 install pyrealsense2
```

## Installing

To install the image_processing package in your catkin workspace, you will need to run the following lines:
```
cd catkin_ws/src
git clone https://github.com/RoboHubEindhoven/suii.git
catkin_make
```

## Running the tests

For testing the BlasterX Sens3D run the [realsense_test.py](https://github.com/RoboHubEindhoven/suii/tree/master/suii_vision/scripts/camera_calibration) and check if the camera feed opens.

In the code post_processing_v2.py you need to change the loaded data in the __init__ to your own used path. currently it is:
```
data = np.load('/home/suii/catkin_ws/src/image_processing/camera_calibration/mtx.npz')
self.mtx = data['mtx']
data = np.load('/home/suii/catkin_ws/src/image_processing/camera_calibration/dist.npz')
self.dist = data['dist'] 
```

After chaning the lines of code to the correct path, you can run the code using the following commands:

```
cd /home/jeroen/catkin_ws/src/suii/suii_vision/scripts/vision_core
python2 post_processing_test.py 
```
You should recieve some output like:
```
area 6 blur 126 lower 75 upper 251
pix length: 77.9999847412, pix width: 56.9999923706
Object lenght: 48.7499904633 mm, Object width: 35.6249952316 mm
True
[['Bolt', -175.62500476837158, 108.43750476837158, 0.6531277687845073]]
True
```

**note**: If you want to show the processed image, edit the post_processing_test.py file. **change** *build_center = self.test.build_center("Bolt",(0,0,640,480),frame,False)* **to** *build_center = self.test.build_center("Bolt",(0,0,640,480),frame,True)*


## Authors

**Jeroen Bongers and Mike van Lieshout** - *in name of RoboHub Eindhoven* - [RoboHub Eindhoven website](https://robohub-eindhoven.nl/)

