#!/usr/bin/env python3
import rospy
import math
import cv2
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import splprep, splev  # 3차 보간에 사용

class GridSlidingWindow:
    def __init__(self):
        self.occ_grid = None
        self.occ_grid_img = None
        self.img_height = None  # 원본 occupancy grid의 height
        self.img_width = None   # 원본 occupancy grid의 width
        self.occ_grid_resolution = None
        self.origin_x = 0.0  # OccupancyGrid의 원점 x
        self.origin_y = 0.0  # OccupancyGrid의 원점 y

        self.window_width = 15
        self.window_height = 8
        self.min_pixel_threshold = 1
        self.max_empty_windows = 5
        
        self.local_path_pub = rospy.Publisher("local_path", Path, queue_size=1)
        rospy.Subscriber("occupancy_grid", OccupancyGrid, self.occupancy_grid_callback)

    def convert_processed_to_original(self, point):
        x_proc, y_proc = point
        x_orig = self.img_width - 1 - y_proc
        y_orig = self.img_height - 1 - x_proc
        return (x_orig, y_orig)

    def occupancy_grid_callback(self, msg):
        self.occ_grid = msg 
        self.occ_grid_resolution = msg.info.resolution
        self.img_height = msg.info.height
        self.img_width = msg.info.width
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # 회전된 이미지 좌표계로 lane point 검출
        self.occ_grid_img, self.left_lane_points, self.right_lane_points = self.process_occupancy_grid(self.occ_grid)

        if self.left_lane_points.shape[0] == 0 or self.right_lane_points.shape[0] == 0:
            return
    
        # 회전된 좌표계에서 검출된 포인트들을 원래 occupancy grid 좌표계 (grid index)로 변환
        left_orig = np.array([self.convert_processed_to_original(pt) for pt in self.left_lane_points])
        right_orig = np.array([self.convert_processed_to_original(pt) for pt in self.right_lane_points])
        # print("img coord", self.left_lane_points, ">>>")
        # print("grid_map_coord", left_orig,">>>")
        print("img coord", self.right_lane_points, ">>>")
        print("grid_map_coord", right_orig,">>>")
        # 원래 좌표계에서 x 좌표 기준 정렬 (낮은 값부터)
        left_orig = left_orig[left_orig[:, 0].argsort()]
        # print(left_orig)
        right_orig = right_orig[right_orig[:, 0].argsort()]
        # print(left_orig[-1][0] , "left orig의 x좌표 맨 마지막")

        # 원래 grid index 값을 기반으로 Path 생성 (origin 보정 및 해상도 적용)
        raw_waypoints = []  # (x, y) 좌표 리스트
        # 좌측과 우측 포인트를 동시에 순회
        left_idx, right_idx = 0, 0
        while left_idx < len(left_orig) and right_idx < len(right_orig):
            avg_x = (left_orig[left_idx][0] + right_orig[right_idx][0]) / 2
            avg_y = (left_orig[left_idx][1] + right_orig[right_idx][1]) / 2
            x = self.origin_x + avg_x * self.occ_grid_resolution - 0.25
            y = self.origin_y + avg_y * self.occ_grid_resolution
            raw_waypoints.append((x, y))
            left_idx += 1
            right_idx += 1

        while left_idx < len(left_orig):
            avg_x = (left_orig[left_idx][0] + right_orig[-1][0]) / 2
            avg_y = (left_orig[left_idx][1] + right_orig[-1][1]) / 2
            x = self.origin_x + avg_x * self.occ_grid_resolution - 0.25
            y = self.origin_y + avg_y * self.occ_grid_resolution
            raw_waypoints.append((x, y))
            left_idx += 1

        while right_idx < len(right_orig):
            avg_x = (left_orig[-1][0] + right_orig[right_idx][0]) / 2
            avg_y = (left_orig[-1][1] + right_orig[right_idx][1]) / 2
            x = self.origin_x + avg_x * self.occ_grid_resolution - 0.25
            y = self.origin_y + avg_y * self.occ_grid_resolution
            raw_waypoints.append((x, y))
            right_idx += 1

        # 보간 전에 (0,0)을 무조건 리스트 맨 앞에 추가
        raw_waypoints.insert(0, (0.0, 0.0))
        raw_waypoints = np.array(raw_waypoints)

        # 3차 스플라인 보간: 충분한 점(최소 4개)일 경우에 적용
        if len(raw_waypoints) >= 4:
            # x, y 좌표를 분리
            tck, u = splprep([raw_waypoints[:,0], raw_waypoints[:,1]], s=0, k=3)
            # 보간할 점의 개수 (예: 100개)
            u_new = np.linspace(0, 1, num=100)
            x_new, y_new = splev(u_new, tck)
        else:
            # 보간할 점이 부족하면 기존 점 그대로 사용
            x_new, y_new = raw_waypoints[:,0], raw_waypoints[:,1]

        # 보간된 결과를 기반으로 Path 메시지 생성
        interpolated_path = Path()
        interpolated_path.header.frame_id = 'map'
        interpolated_path.header.stamp = rospy.Time.now()

        for x, y in zip(x_new, y_new):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = Quaternion(0, 0, 0, 1)
            interpolated_path.poses.append(pose_stamped)

        self.local_path_pub.publish(interpolated_path)

    def sliding_window_lane_detection_left(self, img_half, window_width, window_height, min_pixel_threshold):
        # img_half는 단일 채널 8비트 이미지 (CV_8U)여야 함
        out_img = cv2.cvtColor(img_half, cv2.COLOR_GRAY2BGR)
        height, width = img_half.shape

        # 역이진화: occupied 픽셀(점유된 영역)을 검출
        ret, binary = cv2.threshold(img_half, 127, 255, cv2.THRESH_BINARY_INV)
        nonzero = cv2.findNonZero(binary)
        if nonzero is None:
            return [], out_img

        nonzero = nonzero.reshape(-1, 2)
        start_y = height - 1 - window_height
        min_y_occupied = int(np.min(nonzero[:, 1]))

        bottom_indices = np.where(nonzero[:, 1] >= start_y - 5)[0]
        if len(bottom_indices) > 0:
            current_x = int(np.mean(nonzero[bottom_indices, 0]))
        else:
            current_x = width // 2

        total_windows = 0         # 전체 창 개수 카운터
        valid_lane_points = []    # 유효한 창(녹색 창)의 중심 좌표 리스트
        invalid_lane_points = [] # 유효하지 않은 창(빨간색)의 중심 좌표 리스트
        total_lane_points = []
        current_y = start_y 
        consecutive_empty = 0

        while True:
            if current_y - window_height < min_y_occupied or current_y < 0:
                break

            win_y_low = current_y - window_height
            win_y_high = current_y
            win_x_low = max(0, current_x - window_width // 2)
            win_x_high = min(width, current_x + window_width // 2)

            window_img = img_half[win_y_low:win_y_high, win_x_low:win_x_high]
            ret, window_binary = cv2.threshold(window_img, 127, 255, cv2.THRESH_BINARY_INV)
            nonzero_window = cv2.findNonZero(window_binary)

            total_windows += 1  # 전체 창 카운터 증가

            if nonzero_window is not None and len(nonzero_window) >= min_pixel_threshold:
                nonzero_window = nonzero_window.reshape(-1, 2)
                current_x = win_x_low + int(np.mean(nonzero_window[:, 0]))
                consecutive_empty = 0
                color = (0, 255, 0)
                center_y = (win_y_low + win_y_high) // 2
                valid_lane_points.append((current_x, center_y))
            else:
                consecutive_empty += 1
                color = (0, 0, 255)
                invalid_lane_points.append((current_x, current_y))
                if consecutive_empty >= self.max_empty_windows:
                    # 일정 개수 이상의 연속된 무효 창이 있으면 종료 맨 위에 올라갈 곳이 없다고 판단되면 버림
                    invalid_lane_points = invalid_lane_points[:-self.max_empty_windows]
                    break

            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), color, 2)
            current_y -= window_height
        total_lane_points = valid_lane_points + invalid_lane_points
        # print("L전체 창 개수:", total_windows, "L초록창 개수:", len(valid_lane_points), "L빨간창 개수", len(invalid_lane_points))
        if len(total_lane_points) == 0: 
            win_y_low = start_y - window_height
            win_y_high = start_y
            win_x_low = max(0, width // 2 - window_width // 2)
            win_x_high = min(width, width // 2+ window_width // 2)
            color = (255, 0, 0)
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), color, 2)
            # print("L창이 없어서 중앙으로")
            total_lane_points.append((width // 2, start_y))
        return total_lane_points, out_img

    def sliding_window_lane_detection_right(self, img_half, window_width, window_height, min_pixel_threshold):
        # img_half는 단일 채널 8비트 이미지 (CV_8U)여야 함
        out_img = cv2.cvtColor(img_half, cv2.COLOR_GRAY2BGR)
        height, width = img_half.shape

        # 역이진화: occupied 픽셀(점유된 영역)을 검출
        ret, binary = cv2.threshold(img_half, 127, 255, cv2.THRESH_BINARY_INV)
        nonzero = cv2.findNonZero(binary)
        if nonzero is None:
            return [], out_img

        nonzero = nonzero.reshape(-1, 2)
        start_y = height - 1 - window_height
        min_y_occupied = int(np.min(nonzero[:, 1]))

        bottom_indices = np.where(nonzero[:, 1] >= start_y - 5)[0]
        if len(bottom_indices) > 0:
            current_x = int(np.mean(nonzero[bottom_indices, 0]))
        else:
            current_x = width // 2

        total_windows = 0         # 전체 창 개수 카운터
        valid_lane_points = []    # 유효한 창(녹색 창)의 중심 좌표 리스트
        invalid_lane_points = [] # 유효하지 않은 창(빨간색)의 중심 좌표 리스트
        total_lane_points = []
        current_y = start_y 
        consecutive_empty = 0

        while True:
            if current_y - window_height < min_y_occupied or current_y < 0:
                break

            win_y_low = current_y - window_height
            win_y_high = current_y
            win_x_low = max(0, current_x - window_width // 2)
            win_x_high = min(width, current_x + window_width // 2)

            window_img = img_half[win_y_low:win_y_high, win_x_low:win_x_high]
            ret, window_binary = cv2.threshold(window_img, 127, 255, cv2.THRESH_BINARY_INV)
            nonzero_window = cv2.findNonZero(window_binary)

            total_windows += 1  # 전체 창 카운터 증가

            if nonzero_window is not None and len(nonzero_window) >= min_pixel_threshold:
                nonzero_window = nonzero_window.reshape(-1, 2)
                current_x = win_x_low + int(np.mean(nonzero_window[:, 0]))
                consecutive_empty = 0
                color = (0, 255, 0)
                center_y = (win_y_low + win_y_high) // 2
                valid_lane_points.append((current_x, center_y))
            else:
                consecutive_empty += 1
                color = (0, 0, 255)
                invalid_lane_points.append((current_x, current_y))
                if consecutive_empty >= self.max_empty_windows:
                    invalid_lane_points = invalid_lane_points[:-self.max_empty_windows]
                    # 일정 개수 이상의 연속된 무효 창이 있으면 종료
                    break

            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), color, 2)
            current_y -= window_height
        total_lane_points = valid_lane_points + invalid_lane_points
        # print("R전체 창 개수:", total_windows, "R초록창 개수:", len(valid_lane_points), "R빨간창 개수", len(invalid_lane_points))
        if len(total_lane_points) == 0:
            # print("R창이 없어서 중앙으로")
            win_y_low = start_y - window_height
            win_y_high = start_y
            win_x_low = max(0, width // 2 - window_width // 2)
            win_x_high = min(width, width // 2+ window_width // 2)
            color = (255, 0, 0)
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), color, 2)
            total_lane_points.append((width // 2, start_y))       
        return total_lane_points, out_img

    def process_occupancy_grid(self, msg: OccupancyGrid):
        # OccupancyGrid 데이터를 (height, width) shape의 numpy 배열로 변환
        grid_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        # 0~100 값을 0~255 범위로 변환 후 반전 (점유된 곳은 어둡게)
        img = 255 - (grid_data * 255 // 100).astype(np.uint8)
        # y축 반전: 원래 OccupancyGrid의 아래쪽이 원점일 수 있으므로
        grid_data_flipped = img[::-1]
        # 행렬 전치 후에 행을 뒤집기
        processed_img = np.transpose(grid_data_flipped)[::-1]
        # 회전된 이미지에서 좌우 분할 (실제 열 크기를 기준)
        rotated_mid_x = processed_img.shape[1] // 2
        left_img = processed_img[:, :rotated_mid_x]
        right_img = processed_img[:, rotated_mid_x:]
        
        left_lane_points, left_vis = self.sliding_window_lane_detection_left(left_img, self.window_width, self.window_height, self.min_pixel_threshold)
        # if len(left_lane_points): print(left_lane_points[-1],"좌측 맨 뒤 좌표")
        right_lane_points, right_vis = self.sliding_window_lane_detection_right(right_img, self.window_width, self.window_height, self.min_pixel_threshold)
        # if len(right_lane_points): print(right_lane_points[-1],"우측 맨 뒤 좌표")
        left_lane_points = np.array(left_lane_points)
        right_lane_points = np.array(right_lane_points)

        # 우측 이미지 좌표는 회전된 이미지 기준이므로 x 좌표에 offset 적용
        if right_lane_points.shape[0] > 0:
            right_lane_points[:, 0] += rotated_mid_x

        combined_vis = np.zeros((processed_img.shape[0], processed_img.shape[1], 3), dtype=np.uint8)
        combined_vis[:, :rotated_mid_x] = left_vis
        combined_vis[:, rotated_mid_x:] = right_vis
        # print(combined_vis.shape)
        expanded_img = cv2.resize(combined_vis, (350, 500), interpolation=cv2.INTER_LINEAR)
        cv2.imshow("Occupancy Map with ROI and Sliding Windows", expanded_img)
        cv2.waitKey(1)
        
        return processed_img, left_lane_points, right_lane_points


if __name__ == '__main__':
    rospy.init_node("grid_sliding_window", anonymous=True)
    grid_sliding_window = GridSlidingWindow()
    rospy.spin()
