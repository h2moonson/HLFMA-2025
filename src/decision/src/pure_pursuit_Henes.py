#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[실제 차량용 최종 버전]
실제 차량의 GPS(WGS84)와 인지 정보를 바탕으로 자율주행을 수행합니다.
장애물 정보는 라이다 기준 로컬 좌표로 가정하고 올바르게 변환합니다.
제어 명령은 아두이노가 수신할 수 있는 Twist 메시지 형태로 발행합니다.
"""

import rospy
import numpy as np
import tf

from nav_msgs.msg import Path
from std_msgs.msg import String, Int32, Int64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# utils.py는 실제 차량용으로 별도 관리하거나, 경로가 같다면 공용으로 사용합니다.
from utils import PurePursuit, PidController, PathReader
from utils import compute_s_and_yaw, cartesian_to_frenet, frenet_to_cartesian, generate_quintic_path

# 라이다 인지 노드가 발행하는 메시지 타입 (실제 환경에 맞게 수정)
from lidar_object_detection.msg import ObjectInfo


class EgoStatus:
    """차량의 현재 상태(위치, 헤딩, 속도)를 저장하는 데이터 클래스"""
    def __init__(self):
        self.position = Vector3()
        self.heading = 0.0
        self.velocity = 0.0 # km/h

class AutonomousDriver:
    """자율주행의 모든 판단과 제어를 총괄하는 메인 클래스"""
    def __init__(self):
        rospy.init_node('real_vehicle_driver_node', anonymous=True)

        # Publisher
        self.cmd_vel_publisher = rospy.Publisher('/teleop_cmd_vel', Twist, queue_size=1)
        self.ego_marker_publisher = rospy.Publisher('/ego_marker', Marker, queue_size=1)
        self.waypoint_publisher = rospy.Publisher('/current_waypoint', Int64, queue_size=1)
        self.target_point_publisher = rospy.Publisher('/pure_pursuit_target_point', Marker, queue_size=1)
        self.global_path_publisher = rospy.Publisher('/global_path', Path, queue_size=1) 
        
        # Subscriber
        rospy.Subscriber("/ntrip_rops/ublox_gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/lane_valid", Int32, self.lane_error_callback)
        rospy.Subscriber("/driving_mode", String, self.driving_mode_callback)
        rospy.Subscriber("/obstacle_info", ObjectInfo, self.obstacle_callback)
        rospy.Subscriber("/local_path", Path, self.local_path_callback)

        # [핵심] WGS84 -> UTM-52N 좌표계 변환기
        self.wgs84_to_utm52n = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        # [중요] 차량 중심(base_link)에서 라이다 센서까지의 오프셋 (x:앞쪽+, y:왼쪽+)
        self.lidar_offset_x = 1.0  # 값은 실제 차량에 맞게 측정 후 수정해야 함

        # 경로 파일의 기준이 되는 원점 (UTM-52N)
        self.map_origin = [249947.5672, 3688367.483] # 예: jeju_cw_origin

        # 클래스 멤버 변수 초기화
        self.status = EgoStatus()
        self.current_mode = 'wait'
        self.is_gps_received = False
        self.is_avoiding = False
        self.avoidance_path = Path()
        self.current_waypoint = 0

        # 속도 추정을 위한 변수
        self.prev_gps_time = None
        self.prev_utm_pos = None

        # 제어 파라미터
        self.max_velocity_kph = 10.0
        self.min_velocity_kph = 2.0
        self.lane_steering_deg = 0.0
        self.gps_steering_deg = 0.0
        self.lane_steering_gain_1 = rospy.get_param('~lane_k1', 2.0e-3)
        self.lane_steering_gain_2 = rospy.get_param('~lane_k2', 1.0e-5)
        
        # 제어기 및 경로 객체 생성
        self.pure_pursuit_controller = PurePursuit()
        self.pid_controller = PidController()
        path_reader = PathReader('decision', self.map_origin)
        self.global_path = path_reader.read_txt("clockwise.txt")
        
        # 메인 루프
        rate = rospy.Rate(40) 
        while not rospy.is_shutdown():
            if not self.is_gps_received or self.current_mode == 'wait':
                rate.sleep()
                continue
            
            path_to_follow = self.global_path
            if self.is_avoiding:
                path_to_follow = self.avoidance_path
                if self.is_avoidance_complete():
                    self.is_avoiding = False
                    path_to_follow = self.global_path

            self.current_waypoint, local_path = self.pure_pursuit_controller.findLocalPath(path_to_follow, self.status)
            self.waypoint_publisher.publish(self.current_waypoint)
            
            if len(local_path.poses) < 2:
                rate.sleep()
                continue

            self.pure_pursuit_controller.getPath(local_path)
            self.pure_pursuit_controller.getEgoStatus(self.status)
            
            steer_deg_pp, target_x, target_y, _ = self.pure_pursuit_controller.steeringAngle(0)
            self.gps_steering_deg = steer_deg_pp

            target_velocity_kph = self.min_velocity_kph
            curvature_info = self.pure_pursuit_controller.estimateCurvature()
            if curvature_info:
                corner_theta_degree, _, _ = curvature_info
                target_velocity_kph = self.corner_speed_controller(corner_theta_degree)

            final_target_velocity, final_steering_degree = 0.0, 0.0
            
            if self.current_mode == 'cam':
                final_steering_degree = self.lane_steering_deg
                final_target_velocity = 4.0
            elif self.current_mode == 'gps':
                final_steering_degree = self.gps_steering_deg
                final_target_velocity = target_velocity_kph
            else: 
                self.publish_arduino_command(0, 0, 1.0)
                rate.sleep()
                continue
            
            pid_output = self.pid_controller.pid(final_target_velocity, self.status.velocity)
            brake_cmd = -pid_output if pid_output < 0 else 0.0

            self.publish_arduino_command(final_target_velocity, final_steering_degree, brake_cmd)
            
            self.visualize_target_point(target_x, target_y)
            self.visualize_ego_marker()
            self.global_path_publisher.publish(self.global_path)
            rate.sleep()

    def gps_callback(self, msg):
        if msg.status.status < 0: # GPS 수신 불량
            self.is_gps_received = False
            return

        utm_x, utm_y = self.wgs84_to_utm52n(msg.longitude, msg.latitude)
        current_time = msg.header.stamp.to_sec()

        if self.prev_utm_pos is not None and self.prev_gps_time is not None:
            dt = current_time - self.prev_gps_time
            if dt > 0.01:
                dx, dy = utm_x - self.prev_utm_pos[0], utm_y - self.prev_utm_pos[1]
                distance = np.hypot(dx, dy)
                
                velocity_ms = distance / dt
                self.status.velocity = velocity_ms * 3.6

                if distance > 0.2:
                    self.status.heading = np.degrees(np.arctan2(dy, dx))
        
        self.status.position.x = utm_x - self.map_origin[0]
        self.status.position.y = utm_y - self.map_origin[1]
        
        self.prev_utm_pos, self.prev_gps_time = (utm_x, utm_y), current_time
        self.is_gps_received = True

    def obstacle_callback(self, msg: ObjectInfo):
        if self.is_avoiding: return
        if self.current_mode == 'cam':
            for obstacle in msg.markers:
                obs_local_x = obstacle.pose.position.x + self.lidar_offset_x
                obs_local_y = obstacle.pose.position.y
                if (0 < obs_local_x < 1.5) and (abs(obs_local_y) < 0.5):
                    self.publish_arduino_command(0, 0, 1.0)
                    rospy.logwarn("[Obstacle] CAM Mode: Obstacle in ROI. Stopping.")
                    return
        elif self.current_mode == 'gps':
            is_path_blocked, blocked_idx = self.check_path_collision(msg, self.global_path)
            if is_path_blocked:
                rospy.logwarn(f"[Obstacle] GPS Mode: Path blocked at index {blocked_idx}. Generating path.")
                self.generate_avoidance_path(blocked_idx)

    def check_path_collision(self, obstacle_msg, ref_path):
        VEHICLE_RADIUS, ROI_DISTANCE = 0.7, 15.0
        ego_x, ego_y, ego_yaw_rad = self.status.position.x, self.status.position.y, np.deg2rad(self.status.heading)
        start_idx = self.current_waypoint
        end_idx = min(start_idx + len(ref_path.poses) - 1, start_idx + int(ROI_DISTANCE / 0.2))

        for i in range(start_idx, end_idx):
            path_point = ref_path.poses[i].pose.position
            for obstacle in obstacle_msg.markers:
                obs_local_x = obstacle.pose.position.x + self.lidar_offset_x
                obs_local_y = obstacle.pose.position.y
                
                delta_x = obs_local_x * np.cos(ego_yaw_rad) - obs_local_y * np.sin(ego_yaw_rad)
                delta_y = obs_local_x * np.sin(ego_yaw_rad) + obs_local_y * np.cos(ego_yaw_rad)
                obs_global_x = ego_x + delta_x
                obs_global_y = ego_y + delta_y
                
                obs_radius = max(obstacle.scale.x, obstacle.scale.y) / 2.0
                if np.hypot(path_point.x - obs_global_x, path_point.y - obs_global_y) < (VEHICLE_RADIUS + obs_radius):
                    return True, i
        return False, -1

    def generate_avoidance_path(self, blocked_idx):
        try:
            ref_path = self.global_path
            s_list, yaw_list = compute_s_and_yaw(ref_path)
            s0, l0 = cartesian_to_frenet(self.status.position.x, self.status.position.y, ref_path, s_list)
            AVOIDANCE_LENGTH, LATERAL_OFFSET = 15.0, 2.0
            target_s, target_l = s_list[blocked_idx] + AVOIDANCE_LENGTH, LATERAL_OFFSET
            l_path_func = generate_quintic_path(s0, l0, target_s, target_l)
            new_path = Path(); new_path.header.frame_id = 'map'
            s_points = np.linspace(s0, target_s, int((target_s - s0) / 0.2))
            for s in s_points:
                l = l_path_func(s)
                x, y, yaw = frenet_to_cartesian(s, l, ref_path, s_list, yaw_list)
                pose = PoseStamped(); pose.header.frame_id = 'map'
                pose.pose.position.x, pose.pose.position.y = x, y
                q = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
                new_path.poses.append(pose)
            self.avoidance_path, self.is_avoiding = new_path, True
        except Exception as e:
            rospy.logerr(f"Failed to generate avoidance path: {e}")
            self.is_avoiding = False
    
    def driving_mode_callback(self, msg):
        self.current_mode = msg.data
        if self.current_mode != 'gps': self.is_avoiding = False

    def lane_error_callback(self, msg):
        PIXEL_ERROR = msg.data - 320
        self.lane_steering_deg = np.rad2deg(self.lane_steering_gain_1 * PIXEL_ERROR + self.lane_steering_gain_2 * PIXEL_ERROR * abs(PIXEL_ERROR))

    def local_path_callback(self, msg):
        """/local_path 토픽으로부터 경로를 받아 저장합니다."""
        self.lidar_path = msg 
    
    def publish_arduino_command(self, target_vel_kph, steering_deg, brake_val):
        twist_msg = Twist()
        if brake_val > 0.1:
            twist_msg.linear.x = 0.0
        else:
            scaled_velocity = (target_vel_kph / self.max_velocity_kph) * 255.0
            twist_msg.linear.x = np.clip(scaled_velocity, 0, 255)
        
        # 아두이노가 받는 값 = angular.z * 0.2 라고 가정하면, angular.z = steer_angle / 0.2
        # steer_angle은 라디안 단위여야 함.
        steering_rad = np.deg2rad(steering_deg)
        twist_msg.angular.z = steering_rad / 0.2 # 이 값은 실제 아두이노 코드에 맞춰 튜닝 필요
        self.cmd_vel_publisher.publish(twist_msg)

    def corner_speed_controller(self, corner_theta_degree):
        corner_theta_degree = min(corner_theta_degree, 30)
        target_vel = -0.5 * corner_theta_degree + 10
        return np.clip(target_vel, self.min_velocity_kph, self.max_velocity_kph)
        
    def is_avoidance_complete(self):
        if not self.avoidance_path.poses:
            return False
        dist_to_end = np.hypot(
            self.status.position.x - self.avoidance_path.poses[-1].pose.position.x,
            self.status.position.y - self.avoidance_path.poses[-1].pose.position.y
        )
        if dist_to_end < 1.5:
            rospy.loginfo("[Main Loop] Avoidance complete. Returning to global path.")
            return True
        return False

    def visualize_target_point(self, x, y):
        marker = Marker(); marker.header.frame_id = "map"; marker.type = marker.SPHERE; marker.action = marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5; marker.color.a = marker.color.r = 1.0
        marker.pose.orientation.w = 1.0; marker.pose.position.x, marker.pose.position.y = x, y
        self.target_point_publisher.publish(marker)

    def visualize_ego_marker(self):
        if not self.is_gps_received: return
        marker = Marker(); marker.header.frame_id = "map"; marker.type = marker.SPHERE; marker.action = marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5; marker.color.a = marker.color.b = 1.0
        marker.pose.orientation.w = 1.0; marker.pose.position.x, marker.pose.position.y = self.status.position.x, self.status.position.y
        self.ego_marker_publisher.publish(marker)

if __name__ == '__main__':
    try:
        AutonomousDriver()
    except rospy.ROSInterruptException:
        pass