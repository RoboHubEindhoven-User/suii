# YOLO Object detection 

This is a repository for the YOLO object detection for the Robocup@work competition that RoboHubEindhoven participates in. This repository consists of the [ultralytics python yolov3](https://github.com/ultralytics/yolov3) with some custom adjustments. If you want to learn more of how the YOLO object detection works, please check out our [wiki](https://github.com/RoboHubEindhoven/YOLO_Object_Detection/wiki).

**Below are some of the results on a custom trained database with the Robocup@work objects:**

![1](https://user-images.githubusercontent.com/39261806/58866927-d4df8180-86b9-11e9-82fa-778f1cc614f9.jpeg)

![index](https://user-images.githubusercontent.com/39261806/58866982-ede83280-86b9-11e9-80dd-680a19e43adf.jpeg)

## Getting started

For this project ubuntu 16.04 LTS is used with [CUDA Toolkit 10.1](https://developer.nvidia.com/cuda-downloads).

### Prerequisites

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

### Troubleshoot

It might occure that the error occures that it cannot find opencv installed for python3, this might be due to ROS or other installations. When this happens run in the yolov3_custom folder the following command and retry after:

```bash
source envpy3
```

### Pretrained Weights
Below are the links to the pretrained weights:

* Darknet `*.weights` format: https://pjreddie.com/media/files/yolov3.weights
* PyTorch `*.pt` format: https://drive.google.com/drive/folders/1uxgUBemJVw9wZsdpboYbzUN4bcRhsuAI

## Authors

**Mike van Lieshout** - *in name of RoboHub Eindhoven* - [RoboHub Eindhoven website](https://www.robohub-eindhoven.nl)

