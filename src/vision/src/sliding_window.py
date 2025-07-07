#! /usr/bin/env python
#-*- coding:utf-8 -*-

import os
import cv2
import rospy
import rospkg
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from std_msgs.msg import Int32

class SlidingWindow:
    def __init__(self):
        DEG2RAD = 0.0174532925

        self.threshold, self.nwindows = 0, 12
        self.win_height, self.win_width = 0, 100 // 2
        
        # deg -> rad (pitch, yaw, fov)
        self.pitch = rospy.get_param('pitch', 55) * DEG2RAD # 10도 55 20도 60
        self.yaw = rospy.get_param('yaw', 0) * DEG2RAD
        self.fov = rospy.get_param('fov', 90) * DEG2RAD
        self.alpha = self.fov / 2

        # m (ldh)
        self.l = rospy.get_param('l', 0.) 
        self.d = rospy.get_param('d', 0.) 
        self.h = rospy.get_param('h', 0.94) # 1.49)
        
        self.cv_brdige = CvBridge()
        
        self.ipm = None
        self.ipm_pub = rospy.Publisher('/ipm', Image, queue_size=1)
        
        # @TODO : lane-error가 아닌 그냥 딱 중앙값으로 설정해야 함
        # format : 0000000...0000000 (lane-error, 31bit) | 0 (valid, invalid, 1bit)
        self.lane_valid = Int32()
        self.lane_valid_pub = rospy.Publisher('/lane_valid', Int32, queue_size=1)

        self.sliding_window_pub = rospy.Publisher('/sliding_window', Image, queue_size=1)
        
        rospy.Subscriber('/lane_segment', Image, self.cam_callback)

    # TODO : 여차하면 그냥 직접 warp 찾는 것이 나을 수도
    def cam_callback(self, msg):
        img = self.cv_brdige.imgmsg_to_cv2(msg) if type(msg) == Image else msg

        m2pix = 300 # meter 단위에서 pixel 단위로 넘어가기 위한 계수, 이걸 키우면 ipm 사이즈가 커짐

        d_long = self.h / np.tan(self.pitch)
        d_short = self.h / np.tan(self.pitch + self.alpha)

        k_long = d_long / np.cos(self.alpha)
        k_short = d_short / np.cos(self.alpha)

        pA = np.array([np.cos(self.alpha + self.yaw), np.sin(self.alpha + self.yaw)]) * k_long * m2pix
        pB = np.array([np.cos(self.yaw - self.alpha), np.sin(self.yaw - self.alpha)]) * k_long * m2pix

        pC = np.array([np.cos(self.alpha + self.yaw), np.sin(self.alpha + self.yaw)]) * k_short * m2pix
        pD = np.array([np.cos(self.yaw - self.alpha), np.sin(self.yaw - self.alpha)]) * k_short * m2pix

        long_side = np.linalg.norm(pA - pB)
        short_side = np.linalg.norm(pC - pD)

        top_y_pos = (d_long - d_short) * m2pix
        dst = np.array([[0., 0.],
                        [long_side, 0.],
                        [(long_side + short_side) / 2, top_y_pos],
                        [(long_side - short_side) / 2, top_y_pos]], dtype=np.float32)
        
        img_h, img_w = img.shape[:2]
        # 혹시나 Lane이 너무 안 보이는 경우 위로 shift 하면 됨
        y_shift = 0
        src = np.array([[0., 0.], [img_w, 0.], [img_w, img_h // 2], [0., img_h // 2]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)

        self.ipm = cv2.warpPerspective(img[img_h // 2 - y_shift:img_h - y_shift, :], M, (img_w, img_h // 2))

        self.threshold = int(self.win_width * self.win_height * 0.8) # 80%
        
        crop = self.ipm[:, int((long_side - short_side) / 2) - 50:int((long_side + short_side) / 2) + 50]
        crop = cv2.resize(crop, (crop.shape[1] * 5, crop.shape[0] * 5), interpolation=cv2.INTER_LINEAR)
        rospy.loginfo('{}'.format(crop.shape))

        self.crop = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
        self.win_height = self.crop.shape[0] // self.nwindows
        
        # cv2.imshow('crop', crop)
        self.ipm_pub.publish(self.cv_brdige.cv2_to_imgmsg(crop, encoding='mono8'))

        left_valid_cnt, right_valid_cnt, left_pts, right_pts = self.sliding_window(crop)

        # TODO : 아래 세 변수에 대해 잘 찾아야 함니다
        offset = 50
        get_idx = 8
        valid_threshold = 8

        if left_valid_cnt > valid_threshold and right_valid_cnt > valid_threshold:
            mid = (left_pts[0][get_idx] + right_pts[0][get_idx]) // 2
            self.lane_valid.data = mid * 2 + 1
        
        elif left_valid_cnt > valid_threshold:
            mid = left_pts[0][get_idx] + offset
            self.lane_valid.data = mid * 2 + 1

        elif right_valid_cnt > valid_threshold:
            mid = right_pts[0][get_idx] - offset
            self.lane_valid.data = mid * 2 + 1

        else:
            self.lane_valid.data = 0

        self.lane_valid_pub.publish(self.lane_valid)

        cv2.imshow('crop', self.crop)
        cv2.waitKey(1)
        rospy.loginfo('left_valid_cnt : {} | right_valid_cnt : {}'.format(left_valid_cnt, right_valid_cnt))

    def sliding_window(self, img):
        if img is None:
            return
    
        nonzero = img.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        histogram = np.sum(img[self.win_height:, :], axis=0)

        midpoint = histogram.shape[0] // 2
        left_current = np.argmax(histogram[:midpoint])
        right_current = np.argmax(histogram[midpoint:]) + midpoint

        rospy.loginfo('left {} right {}'.format(left_current, right_current))
        
        left_lane_inds, right_lane_inds = [], []
        left_valid_cnt, right_valid_cnt = 0, 0

        for window in range(self.nwindows):
            win_y_low = img.shape[0] - (window + 1) * self.win_height
            win_y_high = img.shape[0] - window * self.win_height

            win_left_low  = left_current - self.win_width
            win_left_high = left_current + self.win_width

            win_right_low = right_current - self.win_width
            win_right_high = right_current + self.win_width
            
            cv2.rectangle(self.crop, (win_left_low, win_y_low), (win_left_high, win_y_high), (0, 255, 255), 2)
            cv2.rectangle(self.crop, (win_right_low, win_y_low), (win_right_high, win_y_high), (255, 255, 0), 2)

            good_left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                            (nonzero_x >= win_left_low) & (nonzero_x < win_left_high)).nonzero()[0]
            
            good_right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                            (nonzero_x >= win_right_low) & (nonzero_x < win_right_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > self.threshold:
                left_current = int(np.mean(nonzero_x[good_left_inds]))
                cv2.circle(self.crop, (left_current, img.shape[0] - window * self.win_height), 2, (255, 0, 0), thickness=-1)
                left_valid_cnt += 1

            if len(good_right_inds) > self.threshold:
                right_current = int(np.mean(nonzero_x[good_right_inds]))
                cv2.circle(self.crop, (right_current, (win_y_low + win_y_high) // 2), 2, (255, 0, 0), thickness=-1)
                right_valid_cnt += 1

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        left_x, left_y = nonzero_x[left_lane_inds], nonzero_y[left_lane_inds]
        right_x, right_y = nonzero_x[right_lane_inds], nonzero_y[right_lane_inds]

        return left_valid_cnt, right_valid_cnt, (left_x, left_y), (right_x, right_y)

if __name__ == '__main__':
    rospy.init_node('sliding_window')
    node = SlidingWindow()
    rospy.spin()