#! /usr/bin/env python3

import os
import cv2
import rospy
import rospkg
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class TestCamNode:
    def __init__(self):
        rospy.init_node('test_cam_node')

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('vision')

        self.bridge = CvBridge()
        self.cam_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=1)
        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camCallback)

    def camCallback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.cam_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

    def convert(self, img: np.ndarray):
        return self.bridge.cv2_to_imgmsg(img, 'bgr8')

if __name__ == '__main__':
    testCamNode = TestCamNode()
    rospy.spin()
