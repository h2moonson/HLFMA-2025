#! /usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

class IPM:
    def __init__(self):
        DEG2RAD = 0.0174532925

        # deg (pitch, yaw, fov)
        self.extrinsic_angle = np.array([20, 0, 90], dtype=np.float32)
        self.extrinsic_angle *= DEG2RAD

        self.pitch, self.yaw, self.fov = self.extrinsic_angle
        self.alpha = self.fov / 2

        # m (ldh)
        extrinsic_pos = np.array([0, 0, 1.49], dtype=np.float32)
        self.l, self.d, self.h = extrinsic_pos
        
        self.cv_brdige = CvBridge()
        
        self.image_pub = rospy.Publisher('/result', Image, queue_size=1)
        rospy.Subscriber('/detection_result', Image, self.camCallback)


    def camCallback(self, msg: Image):
        img = self.cv_brdige.imgmsg_to_cv2(msg)

        d_long = self.h / np.tan(self.pitch)
        d_short = self.h / np.tan(self.pitch + self.alpha)

        # center_x_far = d_long * np.cos(self.yaw)
        # center_y_far = d_long * np.sin(self.yaw)

        # center_x_near = d_short * np.cos(self.yaw)
        # center_y_near = d_short * np.sin(self.yaw)

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
        
        # # 변환 후 사다리꼴 모양의 네 점 (예시: 위쪽은 좁고 아래쪽은 넓은 사다리꼴)
        # dst = np.float32([[100, 50], [w-100, 50], [w, h], [0, h]])

        # 원근 변환 행렬 계산
        img_h, img_w = img.shape[:2]
        # print(img_h, img_w)
        src = np.array([[0., 0.], [img_w, 0.], [img_w, img_h // 2], [0., img_h // 2]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        
        warped = cv2.warpPerspective(img[img_h//2:, :], M, (int(long_side), int(top_y_pos)))
        self.image_pub.publish(self.cv_brdige.cv2_to_imgmsg(warped, encoding='bgr8'))

if __name__ == '__main__':
    rospy.init_node('morai_cam_node')
    node = IPM()
    rospy.spin()
