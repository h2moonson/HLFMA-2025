#! /usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

class IPM:
    def __init__(self):
        DEG2RAD = 0.0174532925

        # deg -> rad (pitch, yaw, fov)
        self.pitch = rospy.get_param('pitch', 20) * DEG2RAD
        self.yaw = rospy.get_param('yaw', 0) * DEG2RAD
        self.fov = rospy.get_param('fov', 90) * DEG2RAD
        self.alpha = self.fov / 2

        # m (ldh)
        self.l = rospy.get_param('l', 0.) 
        self.d = rospy.get_param('d', 0.) 
        self.h = rospy.get_param('h', 1.49)
        
        self.cv_brdige = CvBridge()
        
        self.image_pub = rospy.Publisher('/ipm', Image, queue_size=1)
        rospy.Subscriber('/lane_segment', Image, self.camCallback)

    def camCallback(self, msg: Image):
        img = self.cv_brdige.imgmsg_to_cv2(msg)

        d_long = self.h / np.tan(self.pitch)
        d_short = self.h / np.tan(self.pitch + self.alpha)

        k_long = d_long / np.cos(self.alpha)
        k_short = d_short / np.cos(self.alpha)

        pA = np.array([np.cos(self.alpha + self.yaw), np.sin(self.alpha + self.yaw)]) * k_long * 100
        pB = np.array([np.cos(self.yaw - self.alpha), np.sin(self.yaw - self.alpha)]) * k_long * 100

        pC = np.array([np.cos(self.alpha + self.yaw), np.sin(self.alpha + self.yaw)]) * k_short * 100
        pD = np.array([np.cos(self.yaw - self.alpha), np.sin(self.yaw - self.alpha)]) * k_short * 100

        long_side = np.linalg.norm(pA - pB)
        short_side = np.linalg.norm(pC - pD)

        # lt, lb, rb, rt
        top_y_pos = (d_long - d_short) * 100
        dst = np.array([[0., 0.],
                        [long_side, 0.],
                        [(long_side + short_side) / 2, top_y_pos],
                        [(long_side - short_side) / 2, top_y_pos]], dtype=np.float32)
        
        # 원근 변환 행렬 계산
        img_h, img_w = img.shape[:2]
        src = np.array([[0., 0.], [img_w, 0.], [img_w, img_h // 2], [0., img_h // 2]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        
        warped = cv2.warpPerspective(img[img_h//2:, :], M, (int(long_side), int(top_y_pos)))
        self.image_pub.publish(self.cv_brdige.cv2_to_imgmsg(warped, encoding='bgr8'))

if __name__ == '__main__':
    rospy.init_node('morai_cam_node')
    node = IPM()
    rospy.spin()
