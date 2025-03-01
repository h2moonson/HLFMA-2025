#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

# 전역 변수로 occupancy grid 정보를 저장
global_occ_grid = None

# 칼만 필터를 위한 전역 변수 (1차원: lane offset)
kalman_initialized = False
filtered_lane_offset = 0.0
P = 1.0   # 초기 오차 공분산
Q = 0.1   # 프로세스 노이즈 분산
R = 0.1   # 측정 노이즈 분산

def occupancy_grid_callback(msg):
    global global_occ_grid
    global_occ_grid = msg  # 최신 occupancy grid map 저장

def latticePlanner(vehicle_state, lookahead_distance, lane_offsets, step, curvature_delta=1.0):
    """
    현재 차량 전방 기준으로 후보 경로를 생성합니다.
    좌우 최외곽 차선은 진행 거리에 따라 곡선(커브)을 그리도록 합니다.
    나머지 차선은 기존과 같이 직선 offset을 사용합니다.
    
    Args:
        vehicle_state: {'x': float, 'y': float, 'heading': float}
        lookahead_distance: 경로 생성 최대 거리
        lane_offsets: 각 차선의 lateral offset 리스트 (예: [-1.2, -0.6, 0.0, 0.6, 1.2])
        step: 경로 포인트 간 간격
        curvature_delta: 최외곽 차선의 추가 곡률 (값이 클수록 곡률이 더 급격해짐)
    
    Returns:
        candidate_paths: 각 경로가 (x, y) 튜플 리스트로 구성된 후보 경로 리스트.
    """
    candidate_paths = []
    num_points = int(lookahead_distance / step)
    start_x = vehicle_state['x']
    start_y = vehicle_state['y']
    theta = vehicle_state['heading']

    leftmost = min(lane_offsets)
    rightmost = max(lane_offsets)

    for offset in lane_offsets:
        path = []
        for i in range(num_points):
            d = i * step
            center_x = start_x + d * math.cos(theta)
            center_y = start_y + d * math.sin(theta)
            
            # 좌우 최외곽 차선은 곡선을 그리도록 함.
            if offset == leftmost:
                ps = offset
                pf = offset - curvature_delta  # 더 큰 음수값으로 이동
                u = d / lookahead_distance  # 0~1 사이의 보간 인자
                cur_offset = ps + (pf - ps) * (3*u*u - 2*u*u*u)
            elif offset == rightmost:
                ps = offset
                pf = offset + curvature_delta  # 더 큰 양수값으로 이동
                u = d / lookahead_distance
                cur_offset = ps + (pf - ps) * (3*u*u - 2*u*u*u)
            else:
                cur_offset = offset

            # 차량 heading에 따른 lateral 방향 (-sin, cos)
            x_offset = cur_offset * -math.sin(theta)
            y_offset = cur_offset * math.cos(theta)
            candidate_x = center_x + x_offset
            candidate_y = center_y + y_offset
            path.append((candidate_x, candidate_y))
        candidate_paths.append(path)
    return candidate_paths

def create_marker_from_path(path_points, marker_id, frame_id="map", color=(0.0, 0.0, 1.0, 1.0), line_width=0.2):
    """
    (x,y) 좌표 리스트를 LINE_STRIP Marker로 변환.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lattice_paths"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = line_width  # 선 두께
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    for (x, y) in path_points:
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = 0.0
        marker.points.append(pt)
    return marker

def world_to_grid(x, y, occ):
    """
    world 좌표 (x, y)를 occupancy grid의 셀 인덱스로 변환.
    occ: OccupancyGrid 메시지
    Returns: (i, j) 인덱스 (행, 열)
    """
    origin_x = occ.info.origin.position.x
    origin_y = occ.info.origin.position.y
    resolution = occ.info.resolution
    j = int((x - origin_x) / resolution)
    i = int((y - origin_y) / resolution)
    return i, j

def compute_path_weight(path_points, occ, occupancy_threshold=50):
    """
    주어진 후보 경로(path_points)의 충돌 패널티를 계산.
    occ: OccupancyGrid 메시지
    occupancy_threshold: 셀 값이 이 값 이상이면 장애물로 간주.
    반환값: 경로의 누적 패널티 값 (낮을수록 장애물이 적음)
    """
    weight = 0.0
    if occ is None:
        return weight
    grid_data = np.array(occ.data).reshape((occ.info.height, occ.info.width))
    for (x, y) in path_points:
        i, j = world_to_grid(x, y, occ)
        if 0 <= i < occ.info.height and 0 <= j < occ.info.width:
            cell_val = grid_data[i, j]
            if cell_val >= occupancy_threshold:
                weight += 1.0
        else:
            weight += 10.0
    return weight

def main():
    global kalman_initialized, filtered_lane_offset, P, Q, R
    rospy.init_node("lattice_path_visualizer", anonymous=True)
    
    rospy.Subscriber("occupancy_grid", OccupancyGrid, occupancy_grid_callback)
    marker_pub = rospy.Publisher("lattice_path_markers", MarkerArray, queue_size=10)
    rate = rospy.Rate(5)  # 5Hz 업데이트

    lane_offsets = [-1.2, -0.6, 0.0, 0.6, 1.2]
    
    while not rospy.is_shutdown():
        vehicle_state = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        # 곡률을 더 급격하게 하기 위해 curvature_delta=1.0 적용
        candidate_paths = latticePlanner(vehicle_state, lookahead_distance=5.0,
                                         lane_offsets=lane_offsets, step=0.5, curvature_delta=1.0)
        
        weights = []
        for path in candidate_paths:
            w = compute_path_weight(path, global_occ_grid, occupancy_threshold=50)
            weights.append(w)
        
        # 장애물로 인한 candidate 선택: weight가 가장 낮은 후보를 측정값으로 사용
        meas_index = int(np.argmin(weights))
        measurement = lane_offsets[meas_index]
        
        # 칼만 필터 업데이트 (1차원)
        if not kalman_initialized:
            filtered_lane_offset = measurement
            P = 1.0
            kalman_initialized = True
        else:
            K = P / (P + R)
            filtered_lane_offset = filtered_lane_offset + K * (measurement - filtered_lane_offset)
            P = (1 - K) * P + Q
        
        # 필터링된 lane offset에 가장 가까운 candidate index 선택
        differences = [abs(lo - filtered_lane_offset) for lo in lane_offsets]
        selected_index = int(np.argmin(differences))
        
        rospy.loginfo("Candidate weights: {}".format(weights))
        rospy.loginfo("Measured lane offset: {:.2f}".format(measurement))
        rospy.loginfo("Filtered lane offset: {:.2f}".format(filtered_lane_offset))
        rospy.loginfo("Selected candidate index: {}".format(selected_index))
        
        # 각 후보 경로를 MarkerArray로 생성 (선택된 경로는 빨간색, 나머지는 파란색)
        marker_array = MarkerArray()
        for i, path in enumerate(candidate_paths):
            if i == selected_index:
                color = (1.0, 0.0, 0.0, 1.0)
                line_width = 0.3
            else:
                color = (0.0, 0.0, 1.0, 1.0)
                line_width = 0.2
            marker = create_marker_from_path(path, marker_id=i, frame_id="map", color=color, line_width=line_width)
            marker_array.markers.append(marker)
        
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    main()
