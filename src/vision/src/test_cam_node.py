#! /usr/bin/env python3

import os
import cv2
import rospy
import rospkg
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class TestCamNode:
    def __init__(self, path='src/YOLOP/inference/videos/1.mp4'):
        rospy.init_node('test_cam_node')

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('vision')  # 'my_pkg'는 패키지 이름
        rospy.loginfo(pkg_path)

        # 패키지 내부 파일 상대 경로 설정
        file_path = os.path.join(pkg_path, path)

        self.bridge = CvBridge()
        self.cam_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)

        self.cap = cv2.VideoCapture(file_path)
       

    def convert(self, img: np.ndarray) -> Image:
        return self.bridge.cv2_to_imgmsg(img, 'bgr8')

if __name__ == '__main__':
    testCamNode = TestCamNode()

    rate = rospy.Rate(30)
    if not testCamNode.cap.isOpened():
        rospy.loginfo('No such video file')
        pass

    else:
        while True:
            ret, img = testCamNode.cap.read()
            if not ret:
                break
            
            testCamNode.cam_pub.publish(testCamNode.convert(img))
            rate.sleep()

    cv2.destroyAllWindows()
