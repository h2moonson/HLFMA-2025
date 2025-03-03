#! /usr/bin/env python3
import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseStamped
from erp_drive.msg import PathReference, PathReferenceElement
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path

import numpy as np
class PathVisualizer:
    def __init__(self):
        rospy.init_node('path')
        
        # 시각화 path
        self.rviz_g_path = Path()
        self.rviz_g_path.header.frame_id = 'map'
        
        self.rviz_g_path2 = Path()
        self.rviz_g_path2.header.frame_id = 'map'
        
        self.rviz_l_path = Path()
        self.rviz_l_path.header.frame_id = 'map'
        
        self.path = PathReference()
        self.path2 = PathReference()
        
        # 시각화용 publisher
        self.rviz_g_path_pub = rospy.Publisher('/rviz_g_path', Path, queue_size=1)
        self.rviz_g_path.header.frame_id = 'map'
        self.rviz_g_path2_pub = rospy.Publisher('/rviz_g_path2', Path, queue_size=1)
        
        # 시각화용 current position publisher
        self.rviz_curr_pose_pub = rospy.Publisher('/rviz_curr_pose', PoseStamped, queue_size=1)
    
        self.start_x = 0
        self.start_y = 0

        self.is_waypoint_set = False
        self.is_local_waypoint_set = False 
        self.setWaypoints("0801_erp_3")

        self.rate = rospy.Rate(30)

        self.curr_pose = PoseStamped()
        self.curr_yaw = 0.
        self.curr_idx = 0

        # 장애물 범위 반지름 
        self.obst_radius = 2.
        self.avoidance_success_radius = 1.
        
        # 경로 상 장애물 체크하는 고정 Ld 값, 차선 너비 설정
        self.block_check_ld = 8.
        self.blocked_idx = 0  #장애물 때문에 막힌 원래 경로 상의 인덱스 

        self.lane_width = 3.
        self.g_tgt_x = 0.
        self.g_tgt_y = 0.        

        self.avoidance_path_mode = 11
        self.local_path = PathReference() # 장애물 회피 경로를 그냥 local_path로
        self.avoidance_in_progress = False # local_path publisher가 아무때나 실행되지 않도록

        self.avoid_mission_idx = 500 #인덱스가 이거 이상부터는 대형 정적 회피로 판단 1차선이라고 set_curr_idx에서 curr_lane = "1차선"으로 변경
        self.curr_lane = None # 주행 차선 정보
        self.prev_lane = None # 회피 경로 생성 후 이전 차선 정보 저장
    def setWaypoints(self, path_name):
        rospy.loginfo('Waypoint setting start')

        rospack = rospkg.RosPack()
        rospack.list()
        self.ROS_HOME = rospack.get_path('erp_drive')
        
        file_path = f'{self.ROS_HOME}/paths/{path_name}.txt'

        self.rviz_g_path.header.stamp = rospy.Time.now()
        self.rviz_g_path2.header.stamp = rospy.Time.now()
        self.rviz_l_path.header.stamp = rospy.Time.now()

        with open(file_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                pose = Pose()
                #------------------------------------옆차선 라인
                pose2 = Pose()
            
                x, y, yaw, mode = map(float, line.split())
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.
                #------------------------------------옆차선 라인
                pose2.position.x = x + 3 * np.cos(yaw + np.pi / 2)
                pose2.position.y = y + 3 * np.sin(yaw + np.pi / 2)
                pose2.position.z = 0.
                
                quat = quaternion_from_euler(0., 0., yaw)
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                #------------------------------------옆차선 라인
                pose2.orientation.x = quat[0]
                pose2.orientation.y = quat[1]
                pose2.orientation.z = quat[2]
                pose2.orientation.w = quat[3]

                element = PathReferenceElement()
                element.pose = pose
                element.mode = int(mode)
                #------------------------------------옆차선 라인
                element2 = PathReferenceElement()
                element2.pose = pose2
                element2.mode = int(mode)
                
                self.path.path.append(element)
                #------------------------------------옆차선 라인
                self.path2.path.append(element2)
        
                if self.start_x == 0 and self.start_y == 0:
                    self.start_x = pose.position.x
                    self.start_y = pose.position.y

                rviz_pose = PoseStamped()
                rviz_pose.header.frame_id = 'map'
                rviz_pose.header.stamp = rospy.Time.now()
                #------------------------------------옆차선 라인
                rviz_pose2 = PoseStamped()
                rviz_pose2.header.frame_id = 'map'
                rviz_pose2.header.stamp = rospy.Time.now()
                #---------------------------------------------
                rviz_pose.pose.position.x = x - self.start_x
                rviz_pose.pose.position.y = y - self.start_y
                rviz_pose.pose.position.z = 0.
                #------------------------------------옆차선 라인
                rviz_pose2.pose.position.x = pose2.position.x - self.start_x
                rviz_pose2.pose.position.y = pose2.position.y - self.start_y
                rviz_pose2.pose.position.z = 0.
                #---------------------------------------------
                rviz_pose.pose.orientation.x = quat[0]
                rviz_pose.pose.orientation.y = quat[1]
                rviz_pose.pose.orientation.z = quat[2]
                rviz_pose.pose.orientation.w = quat[3]
                #------------------------------------옆차선 라인
                rviz_pose2.pose.orientation.x = quat[0]
                rviz_pose2.pose.orientation.y = quat[1]
                rviz_pose2.pose.orientation.z = quat[2]
                rviz_pose2.pose.orientation.w = quat[3]
                #---------------------------------------------

                self.rviz_g_path.poses.append(rviz_pose)
                #------------------------------------옆차선 라인
                self.rviz_g_path2.poses.append(rviz_pose2)
                #---------------------------------------------
        
        self.is_waypoint_set = True

        rospy.loginfo(f'size of waypoint : {len(self.path.path)}')
        
if __name__ == "__main__":
    path = PathVisualizer()
    
    while not rospy.is_shutdown():
        path.rviz_curr_pose_pub.publish(path.curr_pose)
        path.rviz_g_path_pub.publish(path.rviz_g_path)
        path.rviz_g_path2_pub.publish(path.rviz_g_path2)