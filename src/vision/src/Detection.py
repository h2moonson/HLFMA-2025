import argparse
import os, sys
import shutil
import time
from pathlib import Path

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

print(sys.path)

from rospy import loginfo

import cv2
import torch
import torch.backends.cudnn as cudnn

from numpy import random
import numpy as np
import torchvision.transforms as transforms

from YOLOP.lib.config import cfg
from YOLOP.lib.utils.utils import create_logger, select_device, time_synchronized
from YOLOP.lib.utils.augmentations import letterbox_for_img
from YOLOP.lib.models import get_net
from YOLOP.lib.core.general import non_max_suppression, scale_coords
from YOLOP.lib.utils import plot_one_box,show_seg_result
from YOLOP.lib.core.function import AverageMeter

normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

class Detection:
    def __init__(self, cfg, opt):
        logger, _, _ = create_logger(
        cfg, cfg.LOG_DIR, 'test')

        self.opt = opt

        self.device = select_device(logger,opt.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = get_net(cfg)
        checkpoint = torch.load(opt.weights, map_location = self.device)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model = self.model.to(self.device)
        if self.half:
            self.model.half()  # to FP16

        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]

        img = torch.zeros((1, 3, opt.img_size, opt.img_size), device=self.device)  # init img
        _ = self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None  # run once
        self.model.eval()

    def __detect(self, img0: np.ndarray) -> np.ndarray:
        # Run inference
        t0 = time.time()

        inf_time = AverageMeter()
        nms_time = AverageMeter()
        
        h0, w0 = img0.shape[:2]

        img, _, pad = letterbox_for_img(img0, new_shape=self.img_size, auto=True)
        h, w = img.shape[:2]
        shapes = (h0, w0), ((h / h0, w / w0), pad)

        img = np.ascontiguousarray(img)

        img = transform(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        det_out, da_seg_out,ll_seg_out = self.model(img)
        t2 = time_synchronized()

        inf_out, _ = det_out
        inf_time.update(t2-t1,img.size(0))

        # Apply NMS
        t3 = time_synchronized()
        det_pred = non_max_suppression(inf_out, conf_thres=self.opt.conf_thres, iou_thres=self.opt.iou_thres, classes=None, agnostic=False)
        t4 = time_synchronized()

        nms_time.update(t4-t3,img.size(0))
        det=det_pred[0]
        
        _, _, height, width = img.shape
        pad_w, pad_h = shapes[1][1]
        pad_w = int(pad_w)
        pad_h = int(pad_h)
        ratio = shapes[1][0][1]

        da_predict = da_seg_out[:, :, pad_h:(height - pad_h),pad_w:(width - pad_w)]
        da_seg_mask = torch.nn.functional.interpolate(da_predict, scale_factor=int(1/ratio), mode='bilinear')
        _, da_seg_mask = torch.max(da_seg_mask, 1)
        da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
        
        ll_predict = ll_seg_out[:, :,pad_h:(height - pad_h),pad_w:(width - pad_w)]
        ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=int(1/ratio), mode='bilinear')
        _, ll_seg_mask = torch.max(ll_seg_mask, 1)
        ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()

        img_det = show_seg_result(img0, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)

        if len(det):
            det[:,:4] = scale_coords(img.shape[2:],det[:,:4],img_det.shape).round()
            for *xyxy,conf,cls in reversed(det):
                label_det_pred = f'{self.names[int(cls)]} {conf:.2f}'
                plot_one_box(xyxy, img_det , label=label_det_pred, color=self.colors[int(cls)], line_thickness=2)

        loginfo('Done. (%.3fs)' % (time.time() - t0))
        loginfo('inf : (%.4fs/frame)   nms : (%.4fs/frame)' % (inf_time.avg,nms_time.avg))
        
        return img_det

    def detect(self, img0: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            return self.__detect(img0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='weights/End-to-end.pth', help='model.pth path(s)')
    parser.add_argument('--source', type=str, default='inference/videos', help='source')  # file/folder   ex:inference/images
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--save-dir', type=str, default='inference/output', help='directory to save results')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    opt = parser.parse_args()
