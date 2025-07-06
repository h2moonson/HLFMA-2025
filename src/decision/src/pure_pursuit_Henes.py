#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import tf

from nav_msgs.msg import Path
from std_msgs.msg import String, Int32, Int64
from sensor_msgs.msg import NavSatFix
# --- [수정] Twist 메시지 임포트 ---
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj
from morai_msgs.msg import EgoVehicleStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from utils import PurePursuit, PidController, PathReader
from utils import compute_s_and_yaw, cartesian_to_frenet, frenet_to_cartesian, generate_quintic_path

# 사용자 환경에 맞는 장애물 메시지 타입으로 수정해야 합니다
from visualization_msgs.msg import MarkerArray as ObjectInfo

class EgoStatus:
    def __init__(self):
        self.heading = 0.0
        self.position = Vector3()
        self.velocity = Vector3()


class PurePursuitNode:
    def __init__(self):
        rospy.init_node('pure_pursuit_for_arduino', anonymous=True)

        # --- [수정] Publisher 변경 ---
        # 기존 CtrlCmd Publisher 주석 처리
        # self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        # 아두이노 연동을 위한 Twist Publisher 추가
        self.cmd_vel_pub = rospy.Publisher('/teleop_cmd_vel', Twist, queue_size=1)
        
        # 나머지 Publisher
        self.ego_marker_pub = rospy.Publisher('/ego_marker', Marker, queue_size=1)
        self.current_waypoint_pub = rospy.Publisher('/current_waypoint', Int64, queue_size=1)
        self.pure_pursuit_target_point_pub = rospy.Publisher('/pure_pursuit_target_point', Marker, queue_size=1)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1) 
        
        # self.map_origin = [302473.4671541687, 4123735.5805772855] # K-City Origin
        self.jeju_cw_origin = [249947.5672, 3688367.483] # UTM변환 시 East, North
        self.jeju_ccw_origin = [249961.5167, 3688380.892] # UTM변환 시 East, North


        self.status_msg = EgoStatus()
        self.current_mode = 'wait'
        self.is_gps = False
        self.is_gps_status = False

        self.lfd = 0.0
        self.target_velocity = 5.0
        self.steering = 0.0
        
        self.max_velocity = 10.0 # kph 단위의 차량 최고 속도, 속도 매핑에 사용
        self.min_velocity = 2.0

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        # Subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.egoStatusCB)
        rospy.Subscriber("/ntrip_rops/ublox_gps/fix", NavSatFix, self.gpsCB)
        rospy.Subscriber("/lane_valid", Int32, self.calcLaneDetectSteeringCB)
        rospy.Subscriber("/driving_mode", String, self.modeCB)
        rospy.Subscriber("/obstacle_info", ObjectInfo, self.obstacle_callback)

        self.steering_offset = 0.03
        self.current_velocity = 0
        
        self.lane_k1 = rospy.get_param('~lane_k1', 2.0e-3)
        self.lane_k2 = rospy.get_param('~lane_k2', 1.0e-5)
        self.lane_steering = 0.0
        
        self.pp_global = PurePursuit()
        pid = PidController()

        path_reader = PathReader('decision', self.jeju_cw_origin)
        self.cw_path = path_reader.read_txt("clockwise.txt")
        self.ccw_path = path_reader.read_txt("counter_clockwise.txt")

        self.is_avoiding = False
        self.avoidance_path = Path()
        
        rate = rospy.Rate(40) 
        
        while not rospy.is_shutdown():
            self.getEgoCoord()
            
            if self.is_gps_status and self.current_mode != 'wait':
                path_to_follow = self.cw_path
                if self.is_avoiding:
                    path_to_follow = self.avoidance_path
                    if len(path_to_follow.poses) > 0:
                        dist_to_end = np.hypot(
                            self.status_msg.position.x - path_to_follow.poses[-1].pose.position.x,
                            self.status_msg.position.y - path_to_follow.poses[-1].pose.position.y
                        )
                        if dist_to_end < 1.5:
                            rospy.loginfo("[Main Loop] Avoidance complete. Returning to global path.")
                            self.is_avoiding = False
                            path_to_follow = self.cw_path

                self.current_waypoint, local_path = self.pp_global.findLocalPath(path_to_follow, self.status_msg)
                
                if len(local_path.poses) < 2:
                    rospy.logwarn("Local path is too short. Skipping control.")
                    rate.sleep()
                    continue

                self.pp_global.getPath(local_path)
                self.pp_global.getEgoStatus(self.status_msg)
                
                steer, self.target_x, self.target_y, self.lfd = self.pp_global.steeringAngle(0)
                self.steer_gps = (steer + 2.7) * self.steering_offset
                
                curvature_info = self.pp_global.estimateCurvature()
                if curvature_info:
                    corner_theta_degree, _, _ = curvature_info
                    self.target_velocity = self.cornerController(corner_theta_degree)
                
                target_vel = 0.0 # 최종 목표 속도를 저장할 변수
                if self.current_mode == 'cam':
                    steering_msg = self.lane_steering
                    target_vel = 4.0
                elif self.current_mode == 'gps':
                    steering_msg = self.steer_gps
                    target_vel = self.target_velocity
                else: 
                    self.publish_arduino_command(0, 0, 1.0) # 정지 명령
                    rate.sleep()
                    continue

                pid_control_input = pid.pid(target_vel, self.current_velocity)
                brake_msg = 0.0
                if pid_control_input < 0:
                    brake_msg = -pid_control_input

                # --- [수정] 변경된 publisher 함수 호출 ---
                self.publish_arduino_command(target_vel, steering_msg, brake_msg)
                
                self.visualizeTargetPoint()
            
            self.visualizeEgoPoint()
            self.global_path_pub.publish(self.cw_path)
            rate.sleep()
    
    def obstacle_callback(self, msg):
        if self.is_avoiding:
            return

        if self.current_mode == 'cam':
            ego_x = self.status_msg.position.x
            ego_y = self.status_msg.position.y
            ego_yaw_rad = self.status_msg.heading * np.pi / 180.0

            for obstacle in msg.markers:
                obs_x_global = obstacle.pose.position.x
                obs_y_global = obstacle.pose.position.y
                dx = obs_x_global - ego_x
                dy = obs_y_global - ego_y
                local_x = dx * np.cos(ego_yaw_rad) + dy * np.sin(ego_yaw_rad)
                local_y = -dx * np.sin(ego_yaw_rad) + dy * np.cos(ego_yaw_rad)
                if (0 < local_x < 1.0) and (abs(local_y) < 0.3):
                    rospy.logwarn(f"[Obstacle] CAM Mode: Obstacle in ROI detected. Stopping.")
                    self.publish_arduino_command(0, 0, 1.0) # 정지 명령
                    return
            
        elif self.current_mode == 'gps':
            is_path_blocked, blocked_idx = self.check_path_collision(msg, self.cw_path)
            if is_path_blocked:
                rospy.logwarn(f"[Obstacle] GPS Mode: Path blocked at index {blocked_idx}. Generating avoidance path.")
                self.generate_avoidance_path(blocked_idx)

    def check_path_collision(self, obstacle_msg, ref_path):
        vehicle_radius = 0.7 
        roi_distance = 15.0 

        start_idx = self.current_waypoint
        end_idx = min(start_idx + int(roi_distance / 0.2), len(ref_path.poses))

        for i in range(start_idx, end_idx):
            path_point = ref_path.poses[i].pose.position
            for obstacle in obstacle_msg.markers:
                obs_point = obstacle.pose.position
                obs_radius = max(obstacle.scale.x, obstacle.scale.y) / 2.0
                dist = np.hypot(path_point.x - obs_point.x, path_point.y - obs_point.y)
                if dist < (vehicle_radius + obs_radius):
                    return True, i
        return False, -1

    def generate_avoidance_path(self, blocked_idx):
        try:
            ref_path = self.cw_path
            s_list, yaw_list = compute_s_and_yaw(ref_path)
            s0, l0 = cartesian_to_frenet(self.status_msg.position.x, self.status_msg.position.y, ref_path, s_list)
            avoidance_length = 15.0
            lateral_offset = 2.0
            target_s = s_list[blocked_idx] + avoidance_length
            target_l = lateral_offset
            l_path_func = generate_quintic_path(s0, l0, target_s, target_l)
            new_path = Path()
            new_path.header.frame_id = 'map'
            resolution = int((target_s - s0) / 0.2)
            s_points = np.linspace(s0, target_s, resolution)
            for s in s_points:
                l = l_path_func(s)
                x, y, yaw = frenet_to_cartesian(s, l, ref_path, s_list, yaw_list)
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x, pose.pose.position.y = x, y
                q = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
                new_path.poses.append(pose)
            self.avoidance_path = new_path
            self.is_avoiding = True
        except Exception as e:
            rospy.logerr(f"Failed to generate avoidance path: {e}")
            self.is_avoiding = False

    def getEgoCoord(self):
        if self.is_gps:
            self.status_msg.position.x = self.xy_zone[0] - self.map_origin[0]
            self.status_msg.position.y = self.xy_zone[1] - self.map_origin[1]
            self.status_msg.velocity.x = self.current_velocity
            self.is_gps_status = True
        else:
            self.is_gps_status = False

    def gpsCB(self, msg: NavSatFix):
        if msg.status.status == 0: self.is_gps = False; return
        self.is_gps = True
        self.xy_zone = self.proj_UTM(msg.longitude, msg.latitude)
        if hasattr(self, 'prev_xy'):
            dx, dy = self.xy_zone[0] - self.prev_xy[0], self.xy_zone[1] - self.prev_xy[1]
            if np.hypot(dx, dy) > 0.2:
                self.status_msg.heading = np.degrees(np.arctan2(dy, dx))
                self.prev_xy = self.xy_zone
        else: self.prev_xy = self.xy_zone

    def modeCB(self, msg:String): 
        self.current_mode = msg.data
        if self.current_mode != 'gps': self.is_avoiding = False

    def egoStatusCB(self, msg: EgoVehicleStatus): 
        self.current_velocity = max(0, msg.velocity.x * 3.6)

    def calcLaneDetectSteeringCB(self, msg:Int32):
        err_px = msg.data - 320
        steer_rad = self.lane_k1 * err_px + self.lane_k2 * err_px * abs(err_px)
        self.lane_steering = (steer_rad + 2.7) * self.steering_offset
    
    # --- [수정] 아두이노를 위한 Twist 메시지 발행 함수 ---
    def publish_arduino_command(self, target_vel, final_steer, brake):
        """
        계산된 제어값을 아두이노가 이해할 수 있는 Twist 메시지로 변환하여 발행합니다.
        """
        twist_msg = Twist()

        # 속도(linear.x) 매핑
        if brake > 0.1: # 브레이크 값이 일정 이상이면 정지
            twist_msg.linear.x = 0.0
        else:
            # target_vel(kph)을 -255 ~ 255 범위로 스케일링
            # max_velocity는 kph 단위
            scaled_velocity = (target_vel / self.max_velocity) * 255.0
            twist_msg.linear.x = np.clip(scaled_velocity, 0, 255) # 후진은 고려하지 않음

        # 조향(angular.z) 매핑
        # 아두이노: steer_angle = angular.z * 0.2  =>  angular.z = steer_angle / 0.2
        twist_msg.angular.z = final_steer / 0.2

        self.cmd_vel_pub.publish(twist_msg)

    def cornerController(self, corner_theta_degree):
        corner_theta_degree = min(corner_theta_degree, 30)
        target_velocity = -0.5 * corner_theta_degree + 10
        return np.clip(target_velocity, self.min_velocity, self.max_velocity)

    def visualizeTargetPoint(self):
        marker = Marker(); marker.header.frame_id = "map"; marker.type = marker.SPHERE
        marker.action = marker.ADD; marker.scale.x, marker.scale.y, marker.scale.z = 0.5, 0.5, 0.5
        marker.color.a, marker.color.r = 1.0, 1.0; marker.pose.orientation.w = 1.0
        marker.pose.position.x, marker.pose.position.y = self.target_x, self.target_y
        self.pure_pursuit_target_point_pub.publish(marker)

    def visualizeEgoPoint(self):
        marker = Marker(); marker.header.frame_id = "map"; marker.type = marker.SPHERE
        marker.action = marker.ADD; marker.scale.x, marker.scale.y, marker.scale.z = 0.5, 0.5, 0.5
        marker.color.a, marker.color.b = 1.0, 1.0; marker.pose.orientation.w = 1.0
        marker.pose.position.x, marker.pose.position.y = self.status_msg.position.x, self.status_msg.position.y
        self.ego_marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        PurePursuitNode()
    except rospy.ROSInterruptException:
        pass