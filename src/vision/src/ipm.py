#! /usr/bin/env python3

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
        self.win_height, self.win_width = 0, 150 // 2
        
        # deg -> rad (pitch, yaw, fov)
        self.pitch = rospy.get_param('pitch', 20) * DEG2RAD
        self.yaw = rospy.get_param('yaw', 0) * DEG2RAD
        self.fov = rospy.get_param('fov', 65) * DEG2RAD
        self.alpha = self.fov / 2

        # m (ldh)
        self.l = rospy.get_param('l', 0.) 
        self.d = rospy.get_param('d', 0.) 
        self.h = rospy.get_param('h', 0.92) # 1.49)
        
        self.cv_brdige = CvBridge()
        
        self.ipm = None
        self.ipm_pub = rospy.Publisher('/ipm', Image, queue_size=1)
        
        # format : 0000000...0000000 (lane-error)  0 (valid, invalid)
        self.lane_valid = Int32()
        self.lane_valid_pub = rospy.Publisher('/lane_valid', Int32, queue_size=1)

        self.sliding_window_pub = rospy.Publisher('/sliding_window', Image, queue_size=1)
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('vision')
        video_path = os.path.join(pkg_path, 'video', 'output2.mp4')
        cap = cv2.VideoCapture(video_path)

        if not cap.isOpened():
            print("Can't not open video")
            exit()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Play Done.")
                break

            self.cam_callback(frame)
            
            cv2.imshow("ipm", self.ipm)
            # cv2.imshow("Video Playback", frame)

            if cv2.waitKey(30) & 0xFF == 27:
                break

        # rospy.Subscriber('/usb_cam/image_raw', Image, self.cam_callback)
        # rospy.Subscriber('/lane_segment', Image, self.cam_callback)

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
        src = np.array([[0., 0.], [img_w, 0.], [img_w, img_h // 2], [0., img_h // 2]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)

        self.ipm = cv2.warpPerspective(img[img_h // 2:, :], M, (int(long_side), int(top_y_pos)))
        self.win_height = self.ipm.shape[0] // self.nwindows
        self.threshold = int(self.win_width * self.win_height * 0.8) # 80%
        
        crop = self.ipm[:, int((long_side - short_side) / 2):int((long_side + short_side) / 2)]
        cv2.imshow('crop', crop)
        # self.ipm_pub.publish(self.cv_brdige.cv2_to_imgmsg(self.ipm, encoding='bgr8'))

        left_valid_cnt, right_valid_cnt, left_pts, right_pts = self.sliding_window()

    def sliding_window(self):
        if self.ipm is None:
            return
    
        nonzero = self.ipm.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        histogram = np.sum(self.ipm[self.win_height:, :], axis=0)

        midpoint = histogram.shape[0] // 2
        left_current = np.argmax(histogram[:midpoint])
        right_current = np.argmax(histogram[midpoint:]) + midpoint
        
        left_lane_inds, right_lane_inds = [], []
        left_valid_cnt, right_valid_cnt = 0, 0

        for window in range(self.nwindows):
            win_y_low = self.ipm.shape[0] - (window + 1) * self.win_height
            win_y_high = self.ipm.shape[0] - window * self.win_height

            win_left_low  = left_current - self.win_width
            win_left_high = left_current + self.win_width

            win_right_low = right_current - self.win_width
            win_right_high = right_current + self.win_width

            good_left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                            (nonzero_x >= win_left_low) & (nonzero_x < win_left_high)).nonzero()[0]
            
            good_right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                            (nonzero_x >= win_right_low) & (nonzero_x < win_right_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > self.threshold:
                left_current = int(np.mean(nonzero_x[good_left_inds]))
                left_valid_cnt += 1

            if len(good_right_inds) > self.threshold:
                right_current = int(np.mean(nonzero_x[good_right_inds]))
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
