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
from scipy.interpolate import interp1d

DEG2RAD = 1. / 180.* pi
RAD2DEG = 1. / DEG2RAD

class PathReader :  ## 텍스트 파일에서 경로를 출력 ##
    def __init__(self,pkg_name, path_offset):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)
        self.path_offset = path_offset
        rospy.loginfo("path_offset: {}".format(self.path_offset))

    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')
        global_path = Path()
        global_path.header.frame_id='map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split(" ")
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0]) - self.path_offset[0]
            read_pose.pose.position.y=float(tmp[1]) - self.path_offset[1]
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
        self.vehicle_length = 1.04
        self.lfd = 2
        self.min_lfd = 1.0
        self.max_lfd = 6.0
        self.steering = 0

    def getPath(self, msg):
        self.path = msg  #nav_msgs/Path
    
    def getEgoStatus(self, status_msg):
        # status_msg는 AutonomousDriver의 self.status 객체입니다.
        # velocity가 .x를 가진 객체가 아니라 단일 값이므로 바로 사용합니다.
        self.current_vel = status_msg.velocity      # kph
        self.vehicle_yaw = status_msg.heading * DEG2RAD  # rad
        self.current_position.x = status_msg.position.x
        self.current_position.y = status_msg.position.y
        self.current_position.z = 0.0 # z는 0으로 가정
    def steeringAngle(self,_static_lfd = 1.0):
        vehicle_position = self.current_position
        rotated_point = Point()
        self.is_look_forward_point = False

        static_lfd = _static_lfd
        
        for i in self.path.poses:
            path_point = i.pose.position
            dx = path_point.x - vehicle_position.x
            dy = path_point.y - vehicle_position.y
            rotated_point.x = cos(self.vehicle_yaw) * dx + sin(self.vehicle_yaw) * dy
            rotated_point.y = sin(self.vehicle_yaw) * dx - cos(self.vehicle_yaw) * dy

            if rotated_point.x > 0 :
                dis = sqrt(pow(rotated_point.x, 2) + pow(rotated_point.y, 2))
                
                if static_lfd > 0:
                    self.lfd = static_lfd
                else:
                    self.lfd = self.current_vel * 0.9

                    if self.lfd < self.min_lfd: 
                        self.lfd = self.min_lfd 
                    elif self.lfd > self.max_lfd:
                        self.lfd = self.max_lfd

                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
        
        theta = atan2(rotated_point.y, rotated_point.x)
        self.steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd) * RAD2DEG * -1 #deg
        return self.steering, self.forward_point.x, self.forward_point.y, self.lfd

    def findLocalPath(self, ref_path,status_msg):
        out_path=Path()
        current_x=status_msg.position.x
        current_y=status_msg.position.y
        current_waypoint=0
        min_dis=float('inf')

        for i in range(len(ref_path.poses)) :
            dx=current_x - ref_path.poses[i].pose.position.x
            dy=current_y - ref_path.poses[i].pose.position.y
            dis=sqrt(dx*dx + dy*dy)
            if dis < min_dis :
                min_dis=dis
                current_waypoint=i

        if current_waypoint + 80 > len(ref_path.poses) :
            last_local_waypoint= len(ref_path.poses)
        else :
            last_local_waypoint=current_waypoint+80
        
        out_path.header.frame_id='map'
        for i in range(current_waypoint,last_local_waypoint) :
            tmp_pose=PoseStamped()
            tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
            tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
            tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
            tmp_pose.pose.orientation.x=0
            tmp_pose.pose.orientation.y=0
            tmp_pose.pose.orientation.z=0
            tmp_pose.pose.orientation.w=1
            out_path.poses.append(tmp_pose)

        return current_waypoint, out_path

    def estimateCurvature(self):
        if len(self.path.poses) < 3:
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
        return corner_theta * RAD2DEG, self.far_forward_point.x, self.far_forward_point.y

class PidController:
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
        return output

# =====================================================================
# Frenet Frame 관련 신규 추가 함수들
# =====================================================================

def compute_s_and_yaw(ref_path):
    """
    전역 경로의 각 점에 대한 누적 거리(s)와 yaw 값을 계산합니다.
    """
    s_list = [0.0]
    yaw_list = []
    path_points = [p.pose.position for p in ref_path.poses]

    for i in range(1, len(path_points)):
        dx = path_points[i].x - path_points[i-1].x
        dy = path_points[i].y - path_points[i-1].y
        s_list.append(s_list[-1] + np.hypot(dx, dy))
        yaw_list.append(np.arctan2(dy, dx))
    
    if yaw_list:
        yaw_list.append(yaw_list[-1])  # 마지막 yaw 값 복사
    
    return np.array(s_list), np.array(yaw_list)

def cartesian_to_frenet(x, y, ref_path, s_list):
    """
    직교 좌표(x, y)를 Frenet 좌표(s, l)로 변환합니다.
    """
    path_poses = ref_path.poses
    min_dist = float('inf')
    min_idx = 0
    for i, pose in enumerate(path_poses):
        dx = x - pose.pose.position.x
        dy = y - pose.pose.position.y
        dist = dx**2 + dy**2
        if dist < min_dist:
            min_dist = dist
            min_idx = i

    ref_pose = path_poses[min_idx].pose
    ref_x = ref_pose.position.x
    ref_y = ref_pose.position.y

    # Calculate reference yaw
    if min_idx < len(path_poses) - 1:
        next_pose = path_poses[min_idx + 1].pose
        ref_yaw = np.arctan2(next_pose.position.y - ref_y, next_pose.position.x - ref_x)
    else: # Reached the end of the path
        prev_pose = path_poses[min_idx - 1].pose
        ref_yaw = np.arctan2(ref_y - prev_pose.position.y, ref_x - prev_pose.position.x)

    dx = x - ref_x
    dy = y - ref_y

    s = s_list[min_idx]
    l = dx * -np.sin(ref_yaw) + dy * np.cos(ref_yaw)
    return s, l

def frenet_to_cartesian(s, l, ref_path, s_list, yaw_list):
    """
    Frenet 좌표(s, l)를 직교 좌표(x, y, yaw)로 변환합니다.
    """
    path_points = [p.pose.position for p in ref_path.poses]
    # interp1d를 사용하여 s 값에 해당하는 ref_path 상의 점을 보간
    fx = interp1d(s_list, [p.x for p in path_points], fill_value="extrapolate")
    fy = interp1d(s_list, [p.y for p in path_points], fill_value="extrapolate")
    fyaw = interp1d(s_list, yaw_list, fill_value="extrapolate")

    x_ref = fx(s)
    y_ref = fy(s)
    yaw_ref = fyaw(s)

    x = x_ref - l * np.sin(yaw_ref)
    y = y_ref + l * np.cos(yaw_ref)
    
    # 경로의 yaw와 l 방향을 고려하여 최종 yaw 계산 (단순화된 접근)
    final_yaw = yaw_ref 
    return x, y, final_yaw

def generate_quintic_path(s0, l0, s1, l1):
    """
    Quintic Polynomial (5차 다항식) 경로를 생성하는 함수를 반환합니다.
    시작과 끝 지점의 위치, 속도, 가속도가 0이라고 가정합니다.
    """
    A = np.array([
        [1, s0, s0**2,   s0**3,     s0**4,      s0**5],
        [0, 1,  2*s0,    3*s0**2,   4*s0**3,    5*s0**4],
        [0, 0,  2,       6*s0,     12*s0**2,   20*s0**3],
        [1, s1, s1**2,   s1**3,     s1**4,      s1**5],
        [0, 1,  2*s1,    3*s1**2,   4*s1**3,    5*s1**4],
        [0, 0,  2,       6*s1,     12*s1**2,   20*s1**3],
    ])
    b = np.array([l0, 0, 0, l1, 0, 0])
    coeffs = np.linalg.solve(A, b)
    return lambda s: np.polyval(coeffs[::-1], s)