# -*- coding: utf-8 -*-

import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Float64,Int16,Float32MultiArray
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
import tf
import copy

DEG2RAD = 1 / 180 * pi
RAD2DEG = 1 / DEG2RAD

class PathReader :  ## 텍스트 파일에서 경로를 출력 ##
    def __init__(self,pkg_name, path_offset):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)
        self.path_offset = path_offset
        rospy.loginfo(f"path_offset: {self.path_offset}")

    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')
        global_path = Path()
        global_path.header.frame_id='map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split(",")
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[3]) - self.path_offset[0]
            read_pose.pose.position.y=float(tmp[4]) - self.path_offset[1]
            read_pose.pose.position.z=0
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            global_path.poses.append(read_pose)

        openFile.close()
        return global_path 
    

class PurePursuit: ## purePursuit 알고리즘 적용 ##
    def __init__(self):
        self.forward_point = Point()
        self.current_position = Point()
        self.is_look_forward_point = False
        # self.vehicle_length = 4.635 # 4.6 # 3.0
        self.vehicle_length = 1.04 # 4.6 # 3.0

        self.lfd = 2
        self.min_lfd = 1.0
        self.max_lfd = 6.0
        self.steering = 0

    def getPath(self, msg):
        self.path = msg  #nav_msgs/Path 
    
    def getEgoStatus(self, msg):
        self.current_vel = msg.velocity.x  #kph
        self.vehicle_yaw = msg.heading * DEG2RAD  # rad
        self.current_position.x=msg.position.x ## 차량의 현재x 좌표 ##
        self.current_position.y=msg.position.y ## 차량의 현재y 좌표 ##
        # self.current_position.x = 0.0 ## 차량의 현재x 좌표 ##
        # self.current_position.y = 0.0 ## 차량의 현재y 좌표 ##
        self.current_position.z = 0.0 ## 차량의 현재z 좌표 ##


    def findCurrentWaypoint(self, ref_path, status_msg): ## global_path와 차량의 status_msg를 이용해 현재 waypoint 생성 ##
        current_x = status_msg.position.x
        current_y = status_msg.position.y
        current_waypoint = 0
        min_dis = float('inf')

        for i in range(len(ref_path.poses)) :
            dx = current_x - ref_path.poses[i].pose.position.x
            dy = current_y - ref_path.poses[i].pose.position.y
            dis = sqrt(dx * dx + dy * dy)
            if dis < min_dis:
                min_dis = dis
                current_waypoint = i

        return current_waypoint 

    # 타겟 지점 설정하기
    def steeringAngle(self,_static_lfd = 1.0):  ## purePursuit 알고리즘을 이용한 Steering 계산 ## 
        vehicle_position = self.current_position
        rotated_point = Point()
        self.is_look_forward_point = False

        static_lfd = _static_lfd
        rotated_x_threshold = 0.0
        rospy.loginfo(f'len of path: {len(self.path.poses)}')
        rospy.loginfo(f'path: {self.path}')
        for i in self.path.poses: # self.path == local_path 
            path_point = i.pose.position
            # rospy.loginfo(f'path_point: {path_point}')
            # rospy.loginfo(f'vehicle_position: {vehicle_position}')
            dx = path_point.x - vehicle_position.x
            dy = path_point.y - vehicle_position.y
            rotated_point.x = cos(self.vehicle_yaw) * dx + sin(self.vehicle_yaw) * dy
            rotated_point.y = sin(self.vehicle_yaw) * dx - cos(self.vehicle_yaw) * dy

            if rotated_point.x > rotated_x_threshold :
                dis = sqrt(pow(rotated_point.x, 2) + pow(rotated_point.y, 2))

                if static_lfd > 0:
                    self.lfd = static_lfd
                
                if dis >= self.lfd:
                    # rospy.loginfo(f'dis: {dis} | self.lfd: {self.lfd}')
                    self.lfd = self.current_vel * 0.9 # 0309 Morai

                    if self.lfd < self.min_lfd: 
                        self.lfd = self.min_lfd 

                    elif self.lfd > self.max_lfd:
                        self.lfd = self.max_lfd

                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
        
        theta = atan2(rotated_point.y, rotated_point.x)

        # if self.is_look_forward_point:
        self.steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd) * RAD2DEG * -1 #deg
        return self.steering, self.forward_point.x, self.forward_point.y, self.lfd ## Steering 반환 ##
        
        # return 0, 0, 0, 0
        


    def findLocalPath(self, ref_path,status_msg): ## global_path와 차량의 status_msg를 이용해 현재 waypoint와 local_path를 생성 ##
        out_path_control=Path()
        current_x=status_msg.position.x
        current_y=status_msg.position.y
        current_waypoint=0
        min_dis=float('inf')

        waypoint_counts = 80

        for i in range(len(ref_path.poses)) :
            dx=current_x - ref_path.poses[i].pose.position.x
            dy=current_y - ref_path.poses[i].pose.position.y
            dis=sqrt(dx*dx + dy*dy)
            if dis < min_dis :
                min_dis=dis
                current_waypoint=i



        if current_waypoint + waypoint_counts > len(ref_path.poses) :
            last_local_waypoint= len(ref_path.poses)
        else :
            last_local_waypoint=current_waypoint + waypoint_counts

        
        out_path_control.header.frame_id='map'
        for i in range(current_waypoint, last_local_waypoint) :
            if i < 0:
                continue

            tmp_pose=PoseStamped()
            tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
            tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
            tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
            tmp_pose.pose.orientation.x=0
            tmp_pose.pose.orientation.y=0
            tmp_pose.pose.orientation.z=0
            tmp_pose.pose.orientation.w=1
            out_path_control.poses.append(tmp_pose)

        return out_path_control



    def estimateCurvature(self):
        if len(self.path.poses) == 0:
            return None

        vehicle_position = self.current_position
        try:
            last_path_point = self.path.poses[-12].pose.position
        
        except:
            last_path_point = self.path.poses[-1].pose.position

        dx = last_path_point.x - vehicle_position.x
        dy = last_path_point.y - vehicle_position.y

        rotated_point = Point()
        rotated_point.x = cos(self.vehicle_yaw) * dx + sin(self.vehicle_yaw) * dy
        rotated_point.y = sin(self.vehicle_yaw) * dx - cos(self.vehicle_yaw) * dy
    
        self.far_forward_point = last_path_point

        corner_theta = abs(atan2(rotated_point.y, rotated_point.x))
        corner_theta_degree = corner_theta * RAD2DEG

        return corner_theta_degree, self.far_forward_point.x, self.far_forward_point.y

class PidController : ## 속도 제어를 위한 PID 적용 ##
    def __init__(self):
        self.p_gain = 0.7
        self.i_gain = 0.0        
        self.d_gain = 0.05
        self.controlTime = 0.025 
        self.prev_error = 0
        self.i_control = 0

    def pid(self, target_velocity, current_velocity):
        error = target_velocity - current_velocity
        
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error
        if (output == 0):
            print("pid is zero????")
        return output