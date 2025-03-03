#! /usr/bin/env python3

# path 경로를 받기 위함
import os, sys

sys.path.append(os.path.dirname(__file__))

# 경로 관련 모듈
from nav_msgs.msg import Path
from erp_drive.msg import PathReference
from erp_drive.msg import PathReferenceElement

# Pose 등등 위치 관련 모듈
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

# 오일러 <-> 쿼터니언 변환 관련 모듈
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
from scipy.interpolate import CubicHermiteSpline

from std_msgs.msg import Int16
# from lidar_object_detection.msg import ObjectInfo

# fusion 관련
from waypoint_maker.msg import Waypoint

# ros 관련 모듈
import rospy

#utils :: 계산식 모음
from constant import Lane, const
from planner_utils import PathReader, make_avoidance_path, calc_obst_g_coord, is_obst_on_path

from erp_drive.srv import GetStopEvent

class Planner(object):
    def __init__(self, path_name):
        rospy.init_node('planner', argv=path_name)
        
        # 시각화 path
        self.rviz_g_path = Path()
        self.rviz_g_path.header.frame_id = 'map'
        
        self.rviz_g_path2 = Path()
        self.rviz_g_path2.header.frame_id = 'map'
        
        self.rviz_l_path = Path()
        self.rviz_l_path.header.frame_id = 'map'
        
        self.path = PathReference()
        self.path2 = PathReference()
        
        # 시각화용 publisher + global_path
        self.rviz_g_path_pub = rospy.Publisher('/rviz_g_path', Path, queue_size=1)
        self.rviz_g_path.header.frame_id = 'map'
        
        self.rviz_g_path2_pub = rospy.Publisher('/rviz_g_path2', Path, queue_size=1)

        self.g_path_pub = rospy.Publisher('/global_path', PathReference, queue_size=1)
        self.g_path2_pub = rospy.Publisher('/global_path2', PathReference, queue_size=1)
        
        # 시각화용 publisher + local_path (라바콘 한정)
        self.rviz_l_path_pub = rospy.Publisher('/rviz_l_path', Path, queue_size=1)
        self.l_path_pub = rospy.Publisher('/local_path', PathReference, queue_size=1)
        self.l_path_done_signal_pub = rospy.Publisher('/local_path_done', Int16, queue_size=1)
        

        # 시각화용 current position publisher
        self.rviz_curr_pose = rospy.Publisher('/rviz_curr_pose', PoseStamped, queue_size=1)
        self.rviz_curr_pose_pub = rospy.Publisher('/rviz_curr_pose', PoseStamped, queue_size=1)

        self.start_x = 0
        self.start_y = 0
        
        self.is_waypoint_set = False
        self.is_local_waypoint_set = False
        
        self.path_reader = PathReader('erp_drive')
        # self.is_waypoint_set, self.path, self.path2, self.rviz_g_path, self.rviz_g_path2 = self.path_reader.set_waypoints(path_name) 
        # rospy.loginfo(f'global path is set : {self.is_waypoint_set}')

        self.rate = rospy.Rate(30)
        
        # 현재 차량 pose 및 인덱스
        self.stop_line_confirm = False
        self.curr_pose = Pose()
        self.curr_yaw = 0.
        self.curr_idx = 0
        
        self.is_pose_set = False

        self.curr_pose_sub = rospy.Subscriber('/current_pose', Pose, self.curr_pose_CB)
        self.curr_idx_sub = rospy.Subscriber("/curr_idx", Int16, self.set_curr_idx_CB)
        
        self.waypoint_info = rospy.Subscriber('/waypoint_info', Waypoint, self.waypoint_callback)
        
        # 장애물 Bounding Box 정보 Subscriber
        # self.local_path_manager = rospy.Subscriber("/object_info", ObjectInfo , self.local_path_CB)
        
        # 장애물 범위 반지름 
        self.obst_radius = 2.
        self.avoidance_success_radius = 1.
        
        self.avoidance_in_progress = False # local_path publisher가 아무때나 실행되지 않도록
        self.avoid_mission_idx = 500 #인덱스가 이거 이상부터는 대형 정적 회피로 판단 1차선이라고 set_curr_idx에서 curr_lane = "1차선"으로 변경
        
        self.curr_lane = Lane.ONE # 주행 차선 정보
        self.prev_lane = None # 회피 경로 생성 후 이전 차선 정보 저장
        
    def set_curr_idx_CB(self, idx : Int16) :
        self.curr_idx = idx.data
        if self.curr_idx >= self.avoid_mission_idx:
            self.curr_lane = Lane.ONE
        
    def curr_pose_CB(self, msg: Pose):
        self.curr_pose = msg 
        quat = self.curr_pose.orientation 
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        _, _, self.curr_yaw = euler_from_quaternion(orientation_list)


        self.is_pose_set = True
        
    def waypoint_callback(self, msg: Waypoint):
        if not self.is_pose_set: return 

        x_arr, y_arr = msg.x_arr[:msg.cnt], msg.y_arr[:msg.cnt]

        for i in range(len(y_arr)):
            tmp = y_arr[i]
            tmp += -30.0
            y_arr[i] = tmp

        # x_arr, y_arr = (-0.95, ) + msg.x_arr[:msg.cnt], (0, ) + msg.y_arr[:msg.cnt]


        if len(x_arr) <= 1 or len(y_arr) <= 1:
            return

        path_coeff = np.polyfit(x_arr, y_arr, 3)
        path_func = np.poly1d(path_coeff)
        path_yaw = path_func.deriv()

        x = np.linspace(min(x_arr), max(x_arr), 200)
        y = path_func(x)
        yaw = path_yaw(x)

        local_path = PathReference()
        rviz_local_path = Path()
        rviz_local_path.header.frame_id = 'map'

        WHEEL_BASE = 1.04
        LIDAR_GPS_DISTANCE = WHEEL_BASE + 0.

        for i in range(len(x)):
            _x, _y, _yaw = x[i], y[i], yaw[i]
            local_path_element = PathReferenceElement()

            quat = [self.curr_pose.orientation.x, self.curr_pose.orientation.y, self.curr_pose.orientation.z, self.curr_pose.orientation.w]
            _, _, g_yaw = euler_from_quaternion(quat)
            # Test 1 
            # g_yaw -= _yaw

            # local_path_element.pose.position.x = _x + LIDAR_GPS_DISTANCE * np.cos(g_yaw) +
            # local_path_element.pose.position.y = _y + LIDAR_GPS_DISTANCE * np.sin(g_yaw)
            # local_path_element.pose.position.z = 0.
            
            # Test 2
            local_path_element.pose.position.x = (_x + LIDAR_GPS_DISTANCE) * np.cos(g_yaw) - _y * np.sin(g_yaw) + self.curr_pose.position.x
            local_path_element.pose.position.y = (_x + LIDAR_GPS_DISTANCE) * np.sin(g_yaw) + _y * np.cos(g_yaw) + self.curr_pose.position.y

            quat = quaternion_from_euler(0., 0., g_yaw - _yaw)
            local_path_element.pose.orientation.x = quat[0]
            local_path_element.pose.orientation.y = quat[1]
            local_path_element.pose.orientation.z = quat[2]
            local_path_element.pose.orientation.w = quat[3]

            local_path_element.mode = 9
            
            local_path.path.append(local_path_element)

            rviz_local_path_element = PoseStamped()
            rviz_local_path_element.header.frame_id = 'map'
            
            rviz_local_path_element.pose.position.x = (_x + LIDAR_GPS_DISTANCE) * np.cos(g_yaw) - _y * np.sin(g_yaw) + self.curr_pose.position.x
            rviz_local_path_element.pose.position.y = (_x + LIDAR_GPS_DISTANCE) * np.sin(g_yaw) + _y * np.cos(g_yaw) + self.curr_pose.position.y
            rviz_local_path_element.pose.position.z = 0.

            rviz_local_path_element.pose.orientation.x = quat[0]
            rviz_local_path_element.pose.orientation.y = quat[1]
            rviz_local_path_element.pose.orientation.z = quat[2]
            rviz_local_path_element.pose.orientation.w = quat[3]

            rviz_local_path.poses.append(rviz_local_path_element)

        self.rviz_l_path_pub.publish(rviz_local_path)
        # self.l_path_pub.publish(local_path)

        self.stop_signal(self.curr_pose)

    def stop_signal(self, curr_pose: Pose):
        stop_x, stop_y, _ = const.STOP_PLACE_POS[0]
        curr_x, curr_y = curr_pose.position.x, curr_pose.position.y
        
        dist_to_stop = np.hypot(stop_x - curr_x, stop_y - curr_y)
        
        if dist_to_stop < const.STOP_THRESHOLD_DIST and not self.stop_line_confirm:
            rospy.wait_for_service('stop_event')
            stop_event = rospy.ServiceProxy('stop_event', GetStopEvent)
            ret = stop_event(True) # 나 곧 멈춰요
            self.stop_line_confirm = ret.success
        
    def setting_target_point(self, rel_coord: tuple):
        if self.curr_lane == Lane.ONE:
            change =  -1 # 우회전하게
            self.prev_lane = Lane.ONE
        elif self.curr_lane == Lane.TWO: 
            change = 1 #  좌회전 하도록
            self.prev_lane = Lane.TWO
        else : 
            pass 
        
        quat = self.path.path[self.blocked_idx].pose.orientation 
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        _, _, ref_g_yaw = euler_from_quaternion(orientation_list)
        ref_rel_yaw = ref_g_yaw - self.curr_yaw 
        
        # 옆 차선으로 목표 제어점 좌표 생성 (차량에 대한 상대 좌표로 반환)
        x = np.cos(ref_rel_yaw + (np.pi / 2)) * self.lane_width 
        y = np.sin(ref_rel_yaw + (np.pi / 2)) * self.lane_width
        res = (rel_coord[0]+ x* change , rel_coord[1] + y * change , ref_rel_yaw)
        
        self.g_tgt_x = res[0]
        self.g_tgt_y = res[1]
        # self.calculate_midpoint_yaw(res[1], res[2]) 
        return res 

    def avoidance_success_confirm(self):
        dist_to_target = np.hypot(self.curr_pose.position.x - self.g_tgt_x, self.curr_pose - self.g_tgt_y)
        
        if dist_to_target > self.avoidance_success_radius:
            return False
        
        self.is_local_waypoint_set = False

        rospy.loginfo('avoidance_success_confirm() : 목표점 범위 도달 완료, 다시 global 경로로')
        signal = Int16(data = 1)
    
        self.l_path_done_signal_pub.publish(signal)
        self.curr_lane = Lane.ONE if self.prev_lane == Lane.TWO else Lane.TWO
        
        return True
    
    # def local_path_CB(self, msg : ObjectInfo):
    #     # 이미 회피 경로가 생성되어 진행중이라면 pass
    #     if self.avoidance_in_progress:
    #         if self.avoidance_success_confirm():
    #             self.avoidance_in_progress = False 
    #         return 0
        
    #     if self.curr_idx < self.avoid_mission_idx :  
    #         return 0
        
    #     # 하나로 묶고
    #     obstacles = list(zip(msg.centerX, msg.centerY, msg.centerZ, msg.lengthX, msg.lengthY, msg.lengthZ))
        
    #     # 원소가 없으면 return 0
    #     if len(obstacles) == 0:
    #         return 0
        
    #     # 원소가 1개 이상이면 정렬
    #     obstacles.sort()
    #     for obstacle in obstacles: 
    #         g_coord = calc_obst_g_coord(obstacle) # g_coord = (global_x, global_y)
    #         is_path_blocked, self.blocked_idx, self.blocked_coord= is_obst_on_path(self.curr_pose, self.curr_idx, self.path, g_coord)
    #         if is_path_blocked : 
    #             # 회피 경로 생성
    #             rel_tgt_coord = self.setting_target_point(obstacle) # obstacle을 넣을지 blocked_coord를 넣을지는 테스트 해볼것
    #             self.local_path, self.rviz_l_path, self.is_local_waypoint_set, self.avoidance_in_progress = make_avoidance_path(self.curr_pose, rel_tgt_coord) 
    #             break 
        
    def publish_rviz_g_path(self):
        self.g_path_pub.publish(self.path)
        self.g_path2_pub.publish(self.path2)
        
        self.rviz_g_path_pub.publish(self.rviz_g_path)
        self.rviz_g_path2_pub.publish(self.rviz_g_path2)
        
    def publish_rviz_l_path(self):
        self.l_path_pub.publish(self.local_path)
        self.rviz_l_path_pub.publish(self.rviz_l_path)
        

if __name__ == '__main__':
    planner = Planner(sys.argv[1])
    
    while not rospy.is_shutdown():
        if planner.is_waypoint_set:
            planner.publish_rviz_g_path()

        if planner.is_local_waypoint_set:
            planner.publish_rviz_l_path()
        
        planner.rate.sleep()
