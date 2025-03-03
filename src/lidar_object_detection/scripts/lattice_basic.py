#!/usr/bin/env python3
import rospy
import math
import cv2
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
    process_occupancy_grid(msg)

def latticePlanner(vehicle_state, lookahead_distance, lane_offsets, step, curvature_delta=1.0):
    """
    현재 차량 전방 기준으로 후보 경로를 생성합니다.
    중앙 차선을 제외한 나머지 차선은 진행 거리에 따라 곡선(커브)을 그리도록 합니다.
    
    Args:
        vehicle_state: {'x': float, 'y': float, 'heading': float}
        lookahead_distance: 경로 생성 최대 거리
        lane_offsets: 각 차선의 lateral offset 리스트 (예: [-1.2, -0.6, 0.0, 0.6, 1.2])
        step: 경로 포인트 간 간격
        curvature_delta: 곡선의 추가 곡률 (값이 클수록 곡률이 더 급격해짐)
    
    Returns:
        candidate_paths: 각 경로가 (x, y) 튜플 리스트로 구성된 후보 경로 리스트.
    """
    candidate_paths = []
    num_points = int(lookahead_distance / step)
    start_x = vehicle_state['x']
    start_y = vehicle_state['y']
    theta = vehicle_state['heading']

    for offset in lane_offsets:
        path = []
        for i in range(num_points):
            d = i * step
            
            # 첫 번째 포인트는 차량의 시작 위치로 고정
            if i == 0:
                candidate_x = start_x
                candidate_y = start_y
            else:
                center_x = start_x + d * math.cos(theta)
                center_y = start_y + d * math.sin(theta)
                
                # 중앙 차선(0.0)이 아니라면 곡선 형태로 보간 적용
                if offset != 0.0:
                    if offset < 0:
                        ps = offset
                        pf = offset - curvature_delta
                    else:
                        ps = offset
                        pf = offset + curvature_delta
                    u = d / lookahead_distance  # 0~1 사이의 보간 인자
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

def create_path_msg(path_points, frame_id="map"):
    """
    (x, y) 좌표 리스트를 nav_msgs/Path 메시지로 변환합니다.
    """
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = rospy.Time.now()
    for (x, y) in path_points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # 기본 orientation (Quaternion 단위 벡터)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        path_msg.poses.append(pose)
    return path_msg

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

def occupancy_to_image(occ):
    # occupancy grid 데이터를 (height, width) shape로 재구성
    grid_data = np.array(occ.data).reshape((occ.info.height, occ.info.width))
    # 0~100 범위의 occupancy 값을 0~255 그레이스케일로 매핑
    img = 255 - (grid_data * 255 // 100).astype(np.uint8)
    
    # y축 반전: 회전 행렬을 이용하여 수직 플립 수행
    M = np.array([[1, 0, 0],
                  [0, -1, occ.info.height - 1]], dtype=np.float32)
    img_flipped = cv2.warpAffine(img, M, (occ.info.width, occ.info.height))
    
    # 90도 반시계 방향으로 회전
    img_rotated = cv2.rotate(img_flipped, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img_rotated

def sliding_window_lane_detection_side_modified(img, window_width=10, window_height=5, min_pixel_threshold=1):
    """
    새로운 슬라이딩 윈도우 기법:
      - 입력 이미지(한쪽 영역)에서 occupied 픽셀(THRESH_BINARY_INV 적용) 분포를 분석하여,
        이미지 하단에서 시작하는 occupied 픽셀들의 평균 x 좌표를 초기 current_x로 설정합니다.
      - 이후 위쪽으로 창(window)을 쌓으면서, 각 창 내의 occupied 픽셀의 평균 x 좌표로 창의 x 위치를 업데이트합니다.
      - 만약 다음 창의 상단이 해당 영역의 occupied 픽셀 중 가장 위쪽(min y)보다 위라면 창 쌓기를 중단합니다.
      
    Args:
      img: 입력 그레이스케일 이미지 (한쪽 영역)
      window_width, window_height: 각 창의 크기 (픽셀 단위)
      min_pixel_threshold: 창 내에서 occupied 픽셀의 최소 갯수 (1개라도 있으면 업데이트)
      
    Returns:
      lane_points: 각 창의 중심 좌표 (x, y) 리스트
      out_img: 창 영역이 표시된 컬러 이미지 (시각화용)
    """
    out_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    height, width = img.shape

    # 전체 영역에 대해 occupied 픽셀(역이진화: THRESH_BINARY_INV) 검출
    ret, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    nonzero = cv2.findNonZero(binary)
    if nonzero is None:
        return [], out_img

    nonzero = nonzero.reshape(-1, 2)
    # 시작점: y값이 가장 큰(이미지 하단의) occupied 픽셀
    start_y = int(np.max(nonzero[:, 1]))
    # 종료 조건: occupied 픽셀 중 y값이 가장 작은(이미지 상단의) 값
    min_y_occupied = int(np.min(nonzero[:, 1]))

    # 기존에는 단순히 영역 중앙(width//2)에서 시작했으나,
    # 여기서는 하단 근처(예: start_y에서 5픽셀 이내)의 occupied 픽셀들의 평균 x로 초기 current_x 결정
    bottom_indices = np.where(nonzero[:, 1] >= start_y - 5)[0]
    if len(bottom_indices) > 0:
        current_x = int(np.mean(nonzero[bottom_indices, 0]))
    else:
        current_x = width // 2

    lane_points = []
    current_y = start_y

    # 위쪽으로 창을 쌓으며 진행 (y 좌표 감소)
    while True:
        if current_y - window_height < min_y_occupied + 10:
            break

        win_y_low = current_y - window_height
        win_y_high = current_y
        win_x_low = max(0, current_x - window_width // 2)
        win_x_high = min(width, current_x + window_width // 2)

        window_img = img[win_y_low:win_y_high, win_x_low:win_x_high]
        ret, window_binary = cv2.threshold(window_img, 127, 255, cv2.THRESH_BINARY_INV)
        nonzero_window = cv2.findNonZero(window_binary)

        # 만약 창 내 occupied 픽셀이 존재하면 평균 x 좌표로 current_x 업데이트
        if nonzero_window is not None and len(nonzero_window) >= min_pixel_threshold:
            nonzero_window = nonzero_window.reshape(-1, 2)
            current_x = win_x_low + int(np.mean(nonzero_window[:, 0]))

        center_y = (win_y_low + win_y_high) // 2
        lane_points.append((current_x, center_y))

        # 시각화를 위해 창 영역과 중심점을 표시
        cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (222, 255, 0), 2)
        cv2.circle(out_img, (current_x, center_y), 3, (0, 0, 255), -1)

        current_y -= window_height  # 위로 한 칸씩 이동

    return lane_points, out_img

def sliding_window_lane_detection_both(img, window_width=10, window_height=5, min_pixel_threshold=1):
    """
    좌우 영역을 분할한 후, 각 영역에 대해 위에서 정의한 수정된 슬라이딩 윈도우 기법을 적용합니다.
    
    Args:
      img: 입력 그레이스케일 이미지
      window_width, window_height: 각 창의 크기
      min_pixel_threshold: 각 창 내에서 occupied 픽셀의 최소 갯수
      
    Returns:
      left_lane_points: 좌측 영역에서 검출된 차선 중심 좌표 리스트
      right_lane_points: 우측 영역에서 검출된 차선 중심 좌표 리스트
      out_img: 좌우 영역의 결과를 합친 컬러 이미지 (시각화용)
    """
    height, width = img.shape
    mid_x = width // 2
    left_img = img[:, :mid_x]
    right_img = img[:, mid_x:]

    left_lane_points, left_vis = sliding_window_lane_detection_side_modified(left_img, window_width, window_height, min_pixel_threshold)
    right_lane_points, right_vis = sliding_window_lane_detection_side_modified(right_img, window_width, window_height, min_pixel_threshold)

    # 우측 영역의 좌표는 원본 이미지 기준으로 x 좌표에 mid_x를 더함
    right_lane_points = [(x + mid_x, y) for (x, y) in right_lane_points]

    # 좌우 영역의 시각화 결과 결합
    out_img = np.zeros((height, width, 3), dtype=np.uint8)
    out_img[:, :mid_x] = left_vis
    out_img[:, mid_x:] = right_vis

    return left_lane_points, right_lane_points, out_img

def process_occupancy_grid(occ):
    # OccupancyGrid 메시지를 이미지로 변환
    img = occupancy_to_image(occ)
    
    # 예시: 이미지의 하단 50% 영역만 사용 (차량 주변, 가까운 영역)
    height, width = img.shape
    roi_y_start = int(height * 0.5)
    roi = img[roi_y_start:height, :]
    
    # 좌우 영역에 대해 슬라이딩 윈도우 기법 적용
    left_lane_points, right_lane_points, vis_roi = sliding_window_lane_detection_both(roi)
    
    # ROI 영역을 원본 이미지 내의 위치로 복원하려면 y offset 추가
    left_lane_points = [(x, y + roi_y_start) for (x, y) in left_lane_points]
    right_lane_points = [(x, y + roi_y_start) for (x, y) in right_lane_points]
    
    # ROI 결과를 원본 이미지에 배치 (또는 따로 시각화)
    vis_img = img.copy()
    vis_img[roi_y_start:height, :] = cv2.cvtColor(vis_roi, cv2.COLOR_BGR2GRAY)
    
    cv2.imshow("Occupancy Map with ROI and Sliding Windows", vis_img)
    cv2.waitKey(1)
    
    return left_lane_points, right_lane_points

def main():
    global kalman_initialized, filtered_lane_offset, P, Q, R
    rospy.init_node("lattice_path_visualizer", anonymous=True)
    
    rospy.Subscriber("occupancy_grid", OccupancyGrid, occupancy_grid_callback)
    # MarkerArray 대신 Path 메시지를 발행할 퍼블리셔 생성
    path_pub = rospy.Publisher("lattice_paths", Path, queue_size=1)
    # 시각화를 위한 MarkerArray 퍼블리셔 (필요 시 함께 사용 가능)
    marker_pub = rospy.Publisher("lattice_path_markers", MarkerArray, queue_size=1)
    rate = rospy.Rate(5)  # 5Hz 업데이트

    lane_offsets = [-1.2, -0.6, 0.0, 0.6, 1.2]
    
    while not rospy.is_shutdown():
        vehicle_state = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        candidate_paths = latticePlanner(vehicle_state, lookahead_distance=5.0,
                                        lane_offsets=lane_offsets, step=0.5, curvature_delta=1.0)
        
        weights = []
        for path in candidate_paths:
            w = compute_path_weight(path, global_occ_grid, occupancy_threshold=50)
            weights.append(w)
        
        # weight가 모두 같으면 장애물이 없는 상황으로 판단하여 중앙 차선 선택 (lane_offset 0.0)
        if len(set(weights)) == 1:
            measurement = 0.0
        else:
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
        
        selected_path = candidate_paths[selected_index]
        path_msg = create_path_msg(selected_path, frame_id="map")
        path_pub.publish(path_msg)
        
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
