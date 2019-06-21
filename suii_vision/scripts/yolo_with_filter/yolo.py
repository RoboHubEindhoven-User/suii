import time
from sys import platform

from models import *
from utils.datasets import *
from utils.utils import *

defaults_dict = {
    "cfg": "/home/ros/workspace/datasets/yolo_with_filter/config/full_yolo.cfg", # Config file path
    "data": "/home/ros/workspace/datasets/yolo_with_filter/config/full_yolo.data", # Data file path
    "weights": "/home/ros/workspace/datasets/yolo_with_filter/weights/full_yolo.backup", # Weights file path
    "conf_thres": 0.6, # Confidence threshold (accuracy)
    "nms_thres": 0.2,  # Non-maximum supression threshold (compression, lower value = more compression)
    "size": 416 # DO NOT CHANGE!!!
}

class Yolo(object):
    def __init__(self):
        pass

    def load_model(self):
        self._device = torch_utils.select_device()
        self._model = Darknet(defaults_dict["cfg"], defaults_dict["size"])

        # Load defaults_dict["weights"]
        if defaults_dict["weights"].endswith('.pt'):  # pytorch format
            self._model.load_state_dict(torch.load(defaults_dict["weights"], map_location=self._device)['model'])
        else:  # darknet format
            _ = load_darknet_weights(self._model, defaults_dict["weights"])

        # Fuse Conv2d + BatchNorm2d layers
        self._model.fuse()
        # Eval mode
        self._model.to(self._device).eval()

        # Load classes and colors (random)
        self._classes = load_classes(parse_data_cfg(defaults_dict["data"])['names'])
        self._colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self._classes))]
    
    def run(self, img_org, debug):
        img_p = self._single_load(img_org, defaults_dict["size"]) # Load modded image
        t = time.time()
        img_p = torch.from_numpy(img_p).unsqueeze(0).to(self._device)

        # Non-max-supression detection
        pred, _ = self._model(img_p)
        det = non_max_suppression(pred, defaults_dict["conf_thres"], defaults_dict["nms_thres"])[0] 

        rt = []
        
        if det is not None and len(det) > 0:
            # Rescale boxes from 416 to true image size
            det[:, :4] = scale_coords(img_p.shape[2:], det[:, :4], img_org.shape).round()

            if debug:
            # Print results to screen
                print('%gx%g ' % img_p.shape[2:], end='')  # print image size
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()
                    print('%g %ss' % (n, self._classes[int(c)]), end=', ')
            
            # Build detection touple
            for *xyxy, conf, cls_conf, cls in det:
                # Add bbox to the image
                label = '%s %.2f' % (self._classes[int(cls)], conf)
                roi = (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]))
                rt.append((label, roi))

                # Print results to screen
                if debug:
                    plot_one_box(xyxy, img_org, label=label, color=self._colors[int(cls)])
        
        if debug:
            print('Done. (%.3fs)' % (time.time() - t))
            cv2.imshow("capture frame", img_org)
            cv2.waitKey(16)

        return rt

    # Mod raw image 
    def _single_load(self, img, size):
        imr, _, _, _ = letterbox(img, new_shape=size)
        imr = imr[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB
        imr = np.ascontiguousarray(imr, dtype=np.float32)  # uint8 to float32
        imr /= 255.0  # 0 - 255 to 0.0 - 1.0
        return imr
