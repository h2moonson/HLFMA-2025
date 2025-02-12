#! /usr/bin/env python3

import cv2
import rospy
from types import SimpleNamespace

import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import os
import sys
argv = rospy.myargv(argv=sys.argv)

import rospkg

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('vision')  # 'my_pkg'는 패키지 이름
# rospy.loginfo(pkg_path)

sys.path.append(os.path.join(pkg_path, 'src', 'YOLOP'))
# print(sys.path)

from YOLOP.lib.config import cfg

from Detection import Detection

class DetectionNode:
    def __init__(self):
        rospy.init_node("usb_cam_listener")

        opt = SimpleNamespace(
        weights=rospy.get_param("~weights", "weights/End-to-end.pth"),
        source=rospy.get_param("~source", "inference/videos"),
        img_size=rospy.get_param("~img_size", 640),
        conf_thres=rospy.get_param("~conf_thres", 0.25),
        iou_thres=rospy.get_param("~iou_thres", 0.45),
        device=rospy.get_param("~device", "cpu"),
        save_dir=rospy.get_param("~save_dir", "inference/output"),
        augment=rospy.get_param("~augment", False),
        update=rospy.get_param("~update", False),
)
    
        self.detection = Detection(cfg, opt)
        self.bridge = CvBridge()

        self.detection_pub = rospy.Publisher("/detection_result", Image, queue_size=10)

        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
    
    def img_callback(self, msg: Image):
        img0 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res = self.bridge.cv2_to_imgmsg(self.detection(img0))
        self.detection_pub.publish(res)

if __name__ == "__main__":
    detection_node = DetectionNode()

    rospy.spin()
    cv2.destroyAllWindows()
