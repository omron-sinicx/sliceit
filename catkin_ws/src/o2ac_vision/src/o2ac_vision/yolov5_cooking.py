import time
from yolov5.utils.torch_utils import select_device
from yolov5.utils.general import (check_img_size, non_max_suppression_obb, xywh2xyxy)
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.augmentations import letterbox
import os
import sys
from pathlib import Path
from math import pi

import numpy as np
import cv2
import torch
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.rboxs_utils import poly2rbox, rbox2poly

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

class YoloV5Detection:
    def __init__(self,
                 weights='yolov5s.pt',  # model.pt path(s)
                 imgsz=(640, 640),  # inference size (height, width)
                 device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 ):

        # Load model
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights, device=self.device, dnn=False)
        self.stride, self.names, pt, jit, onnx, engine = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx, self.model.engine
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size

        # Half
        # half precision only supported by PyTorch on CUDA
        self.half = (pt or jit or engine) and self.device.type != 'cpu'
        if pt or jit:
            self.model.model.half() if self.half else self.model.model.float()

    def predict(self,
                im=None, # Image input
                im_vis=None,
                ):
        # Padded resize
        img = letterbox(im, self.imgsz[0], stride=self.stride, auto=True)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        # Run inference
        # self.model.warmup(imgsz=(1, 3, *self.imgsz), half=self.half)  # warmup
        im = torch.from_numpy(img).to(self.device)
        im = im.half() if self.half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression_obb(pred,
                conf_thres=0.6,  # confidence threshold
                iou_thres=0.45,  # NMS IOU threshold
                classes=None,  # filter by class: --class 0, or --class 0 2 3
                agnostic=False,  # class-agnostic NMS
                max_det=1000,  # maximum detections per image
                )
        res = []
        
        annotator = Annotator(im_vis, line_width=2, example=str(self.names))
        
        for det in pred:
            bbox = xywh2xyxy(det[:, :4]) # (n, [x1 y1 x2 y2 x3 y3 x4 y4])
            if len(det):
                res.append(torch.cat((bbox, det[:, -3:]), dim=1).cpu().numpy()) # (n, [poly conf cls])
            poly = rbox2poly(det[:, :5])
            for i, obj in enumerate(det):
                c = int(obj[-1])  # integer class
                label = f'{self.names[c]} {obj[-2]:.2f}'
                annotator.poly_label(poly[i], label, color=colors(c, True))
                # annotator.box_label(bbox[i], label, color=colors(c, True))
        
        im_vis = annotator.result()

        return res, im_vis
