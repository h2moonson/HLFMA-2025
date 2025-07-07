#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
[Morai 시뮬레이터 전용 버전]
/Ego_topic을 직접 구독하여 GPS 상태를 판단하고,
주행 모드를 결정하여 /driving_mode 토픽으로 발행합니다.
"""

import rospy
import math
from std_msgs.msg import String, Int32
from collections import deque
# [수정] Morai 메시지 타입 임포트
from morai_msgs.msg import EgoVehicleStatus

class ModeDeciderMorai:
    def __init__(self):
        rospy.init_node('mode_decider_morai', anonymous=True)

        # ───── 파라미터 ─────
        self.window_size        = rospy.get_param('~history_size', 100)
        self.rate_hz            = rospy.get_param('~decide_rate', 10.0)
        # Morai 환경에서는 GPS 유효성 검사가 큰 의미가 없으므로 비율을 낮춰 즉각 반응하게 설정
        self.gps_valid_ratio    = rospy.get_param('~gps_valid_ratio', 0.9)
        self.lane_valid_ratio   = rospy.get_param('~lane_valid_ratio', 1.0)

        # ───── 내부 상태 ─────
        # GPS가 항상 유효하다고 가정하고 시작하지 않기 위해 False(0)로 초기화
        self.mode = 'cam'
        self.lane_history = deque([0] * self.window_size, maxlen=self.window_size)
        self.gps_history  = deque([0] * self.window_size, maxlen=self.window_size)

        # ───── ROS 통신 ─────
        rospy.Subscriber('/lane_valid', Int32, self.lane_cb)
        # [수정] Morai의 Ego_topic을 구독
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.gps_ok_callback)
        self.mode_pub = rospy.Publisher('/driving_mode', String, queue_size=1, latch=True)

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.decide_mode)
        rospy.loginfo("Mode Decider for Morai is running.")

        self.mode_pub.publish(String(data='lidar_only'))

    def lane_cb(self, msg):
        self.lane_history.append(msg.data)

    def gps_ok_callback(self, msg):
        """
        [수정] /Ego_topic을 수신하면 GPS가 유효하다고 판단합니다.
        """
        # 메시지를 잘 받고 있다면 GPS 상태는 양호(True)
        self.gps_history.append(True)

    def _ratio(self, history):
        if not history:
            return 0.0
        # deque에 있는 True(1), False(0)의 평균을 계산
        return sum(history) / float(len(history))

    def decide_mode(self, _):

        
        lane_ok = self._ratio(self.lane_history) >= self.lane_valid_ratio
        gps_ok  = self._ratio(self.gps_history)  >= self.gps_valid_ratio

        new_mode = self.mode # 기본값은 현재 모드 유지

        # FSM (Finite State Machine) 기반 모드 결정 로직
        if lane_ok:
            new_mode = 'cam'
        else: # 차선이 유효하지 않을 때
            if self.mode == 'cam':
                new_mode = 'gps' if gps_ok else 'lidar_only'
            elif self.mode == 'gps':
                # GPS 모드 중 GPS가 끊기면 lidar_only로 전환
                if not gps_ok:
                    new_mode = 'lidar_only'
            elif self.mode == 'lidar_only':
                # lidar_only 모드 중 GPS가 복구되면 gps 모드로 전환
                if gps_ok:
                    new_mode = 'gps'
        
        if new_mode != self.mode:
            rospy.loginfo("[ModeDecider] Mode Changed: {} -> {}".format(self.mode, new_mode))
            self.mode = new_mode
            # self.mode_pub.publish(String(data=self.mode))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ModeDeciderMorai().spin()
    except rospy.ROSInterruptException:
        pass