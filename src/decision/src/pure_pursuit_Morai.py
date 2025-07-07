#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
[Morai 시뮬레이터 최종 버전]
Morai의 UTM-52N 좌표계를 기준으로, 모든 좌표계 변환을 노드 내에서 직접 수행합니다.
장애물 정보는 라이다 기준 로컬 좌표로 들어온다고 가정하고 올바르게 변환합니다.
"""

import rospy
import numpy as np
import tf

from nav_msgs.msg import Path
from std_msgs.msg import String, Int32, Int64
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from geometry_msgs.msg import PoseStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from pyproj import Transformer, CRS

# 라이다 인지 노드가 발행하는 메시지 타입 (실제 환경에 맞게 수정)
from lidar_object_detection.msg import ObjectInfo

# 직접 구현한 계산용 유틸리티 라이브러리
from utils_morai import PurePursuit, PidController, PathReader
from utils_morai import compute_s_and_yaw, cartesian_to_frenet, frenet_to_cartesian, generate_quintic_path

class EgoStatus(object):
    """차량의 현재 상태(위치, 헤딩, 속도)를 저장하는 데이터 클래스"""
    def __init__(self):
        self.position = Vector3()
        self.heading = 0.0
        self.velocity = 0.0

class AutonomousDriver(object):
    def __init__(self):
        rospy.init_node('morai_driver_node', anonymous=True)

        # Publisher
        self.control_publisher = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.ego_marker_publisher = rospy.Publisher('/ego_marker', Marker, queue_size=1)
        self.waypoint_publisher = rospy.Publisher('/current_waypoint', Int64, queue_size=1)
        self.target_point_publisher = rospy.Publisher('/pure_pursuit_target_point', Marker, queue_size=1)
        self.global_path_publisher = rospy.Publisher('/global_path', Path, queue_size=1)
        
        # Subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback)
        rospy.Subscriber("/lane_valid", Int32, self.lane_error_callback)
        rospy.Subscriber("/driving_mode", String, self.driving_mode_callback)
        rospy.Subscriber("/obstacle_info", ObjectInfo, self.obstacle_callback)
        rospy.Subscriber("/local_path", Path, self.local_path_callback)
        
        # [추가] TF 변환을 위한 TransformListener
        self.tf_listener = tf.TransformListener()
        
        # ... (좌표 변환 및 파라미터 초기화는 기존과 동일) ...
        self.morai_offset = [302459.942, 4122635.537]
        self.heading_offset_deg = 45.0 # RViz에서 45도 틀어져 보인다고 하셔서 초기값으로 설정

        self.lidar_offset_x = 1.2

        proj_UTM52N = CRS('EPSG:32652')
        proj_UTMK = CRS('EPSG:5179')
        self.utm52n_to_utmk_transformer = Transformer.from_crs(proj_UTM52N, proj_UTMK, always_xy=True)
        self.map_origin = [935521.5088, 1915824.9469]

        self.lidar_offset_x = 1.2
        
        self.status = EgoStatus()
        self.current_mode = 'wait'

        self.is_status_received = False
        self.is_avoiding = False

        self.avoidance_path = Path()
        self.current_waypoint = 0
        self.lidar_path = Path()

        self.max_velocity_kph = 15.0
        self.min_velocity_kph = 2.0
        self.lane_steering_deg = 0.0
        self.gps_steering_deg = 0.0

        self.lane_steering_gain_1 = rospy.get_param('~lane_k1', 2.0e-3)
        self.lane_steering_gain_2 = rospy.get_param('~lane_k2', 1.0e-5)

        self.ctrl_cmd = CtrlCmd()
        self.ctrl_cmd.longlCmdType = 2

        self.pure_pursuit_controller = PurePursuit()
        self.pid_controller = PidController()

        path_reader = PathReader('decision', self.map_origin)
        self.global_path = path_reader.read_txt("kcity_full_path_1019.txt")

        # 메인 실행 루프
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            if not self.is_status_received or self.current_mode == 'wait':
                rate.sleep()
                continue
            
            local_path = Path()
            
            if self.current_mode == 'lidar_only':
                local_path = self.lidar_path
            
            elif self.current_mode == 'gps':
                path_to_follow = self.global_path
                if self.is_avoiding:
                    path_to_follow = self.avoidance_path
                    if self.is_avoidance_complete():
                        self.is_avoiding = False
                        path_to_follow = self.global_path
                
                self.current_waypoint, local_path = self.pure_pursuit_controller.findLocalPath(path_to_follow, self.status)
                self.waypoint_publisher.publish(self.current_waypoint)

            if len(local_path.poses) < 2:
                rospy.logwarn("[Main] Path too short to follow in mode: {}. Stopping.".format(self.current_mode))
                self.publish_morai_command(0.0, 0.0, 1.0)
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
            elif self.current_mode == 'gps' or self.current_mode == 'lidar_only':
                final_steering_degree = self.gps_steering_deg
                final_target_velocity = target_velocity_kph
            else: 
                self.publish_morai_command(0, 0, 1.0)
                rate.sleep()
                continue
            
            pid_output = self.pid_controller.pid(final_target_velocity, self.status.velocity)
            brake_cmd = -pid_output if pid_output < 0 else 0.0

            self.publish_morai_command(final_target_velocity, final_steering_degree, brake_cmd)
            
            self.visualize_target_point(target_x, target_y)
            self.visualize_ego_marker()
            self.global_path_publisher.publish(self.global_path)
            rate.sleep()


    def ego_status_callback(self, msg):
            """
            [최종 수정본] Morai 시뮬레이터의 좌표계를 경로 좌표계로 변환하고 헤딩을 보정합니다.
            """
            # 1. Morai 시뮬레이터의 로컬 좌표 (x, y)를 받음
            sim_x, sim_y = msg.position.x, msg.position.y

            # 2. 제공된 오프셋을 더해 실제 UTM-52N 좌표로 변환
            vehicle_x_utm52n = sim_x + self.morai_offset[0]
            vehicle_y_utm52n = sim_y + self.morai_offset[1]
            
            # 3. UTM-52N 좌표를 경로와 동일한 UTM-K 좌표로 변환
            vehicle_x_utmk, vehicle_y_utmk = self.utm52n_to_utmk_transformer.transform(
                vehicle_x_utm52n, 
                vehicle_y_utm52n
            )

            # 4. UTM-K 좌표를 경로의 원점(map_origin) 기준으로 하는 로컬 좌표로 변환
            self.status.position.x = vehicle_x_utmk - self.map_origin[0]
            self.status.position.y = vehicle_y_utmk - self.map_origin[1]
            
            # 5. 나머지 상태 정보 업데이트
            self.status.velocity = max(0, msg.velocity.x * 3.6)  # kph
            
            # 6. 헤딩 값에 오프셋을 적용하여 보정
            raw_heading_deg = msg.heading
            corrected_heading_deg = raw_heading_deg - self.heading_offset_deg
            self.status.heading = corrected_heading_deg
            
            # 7. 모든 처리가 성공했으므로, 주행 루프를 시작하도록 플래그를 True로 설정
            self.is_status_received = True


    # --- (이하 나머지 함수들은 수정할 필요가 없습니다) ---

    def obstacle_callback(self, msg):
        if self.is_avoiding: return
        if self.current_mode == 'cam':
            for obstacle in msg.markers:
                obs_local_x = obstacle.pose.position.x + self.lidar_offset_x
                obs_local_y = obstacle.pose.position.y
                if (0 < obs_local_x < 1.5) and (abs(obs_local_y) < 0.5):
                    self.publish_morai_command(0, 0, 1.0)
                    rospy.logwarn("[Obstacle] CAM Mode: Obstacle in ROI. Stopping.")
                    return
        elif self.current_mode == 'gps':
            is_path_blocked, blocked_idx = self.check_path_collision(msg, self.global_path)
            if is_path_blocked:
                rospy.logwarn("[Obstacle] GPS Mode: Path blocked at index {}. Generating path.".format(blocked_idx))
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
            target_s = s_list[blocked_idx] + AVOIDANCE_LENGTH
            target_l = LATERAL_OFFSET
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
            rospy.logerr("Failed to generate avoidance path: {}".format(e))
            self.is_avoiding = False
    
    def driving_mode_callback(self, msg):
        self.current_mode = msg.data
        if self.current_mode != 'gps': self.is_avoiding = False

    def lane_error_callback(self, msg):
        PIXEL_ERROR = msg.data - 320
        steer_rad = self.lane_steering_gain_1 * PIXEL_ERROR + self.lane_steering_gain_2 * PIXEL_ERROR * abs(PIXEL_ERROR)
        self.lane_steering_deg = np.rad2deg(steer_rad)

    def local_path_callback(self, msg):
        """
        [수정] /local_path 토픽으로부터 받은 경로를 'velodyne' -> 'map' 좌표계로 변환합니다.
        """
        try:
            # TF listener를 통해 'map'과 'velodyne' 사이의 변환 관계를 조회
            (trans, rot) = self.tf_listener.lookupTransform('map', 'velodyne', rospy.Time(0))
            
            # 변환 행렬 생성
            transformation_matrix = self.tf_listener.fromTranslationRotation(trans, rot)

            # 변환된 경로를 저장할 새로운 Path 객체
            transformed_path = Path()
            transformed_path.header.frame_id = 'map'

            for pose in msg.poses:
                # 원본 포인트 (velodyne 기준)
                original_point = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 1.0])
                
                # 행렬 곱으로 좌표 변환
                transformed_point = np.dot(transformation_matrix, original_point)

                # 변환된 좌표로 새로운 PoseStamped 생성
                new_pose = PoseStamped()
                new_pose.header.frame_id = 'map'
                new_pose.pose.position.x = transformed_point[0]
                new_pose.pose.position.y = transformed_point[1]
                new_pose.pose.position.z = transformed_point[2]
                new_pose.pose.orientation = pose.pose.orientation # Orientation은 그대로 사용
                transformed_path.poses.append(new_pose)

            # 최종적으로 변환된 경로를 저장
            self.lidar_path = transformed_path

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF transform error in local_path_callback: {}".format(e))

    def publish_morai_command(self, velocity_kph, steering_deg, brake_val):
        self.ctrl_cmd.velocity = velocity_kph
        self.ctrl_cmd.steering = np.deg2rad(steering_deg)
        self.ctrl_cmd.brake = brake_val
        self.control_publisher.publish(self.ctrl_cmd)

    def corner_speed_controller(self, corner_theta_degree):
        corner_theta_degree = min(corner_theta_degree, 30)
        target_vel = -0.5 * corner_theta_degree + 15
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
        if not self.is_status_received: return
        marker = Marker(); marker.header.frame_id = "map"; marker.type = marker.SPHERE; marker.action = marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5; marker.color.a = marker.color.b = 1.0
        marker.pose.orientation.w = 1.0; marker.pose.position.x, marker.pose.position.y = self.status.position.x, self.status.position.y
        self.ego_marker_publisher.publish(marker)

if __name__ == '__main__':
    try:
        AutonomousDriver()
    except rospy.ROSInterruptException:
        pass