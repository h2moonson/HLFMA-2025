#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

class OccupancyGridMapper:
    def __init__(self):
        # 파라미터 설정: 격자 해상도, 크기, 원점 등 (로컬 영역)
        self.resolution = rospy.get_param("~resolution", 0.2)  # 셀 크기 (미터)
        self.width = rospy.get_param("~width", 200)   # 격자 가로 셀 개수
        self.height = rospy.get_param("~height", 200)  # 격자 세로 셀 개수
        self.origin_x = rospy.get_param("~origin_x", -20.0)  # 격자 원점 (global 좌표계에서)
        self.origin_y = rospy.get_param("~origin_y", -20.0)
        self.ground_threshold = rospy.get_param("~ground_threshold", 0.2)  # 지면 제거: z 값이 이 값 이하면 무시

        # OccupancyGrid 메시지 퍼블리셔
        self.pub = rospy.Publisher("occupancy_grid", OccupancyGrid, queue_size=1)
        # Velodyne 포인트 클라우드 구독 (메시지 타입: sensor_msgs/PointCloud2)
        rospy.Subscriber("velodyne_points", PointCloud2, self.pc_callback)
    
    def pc_callback(self, msg):
        # PointCloud2 메시지를 numpy array로 변환 (x, y, z만 추출)
        points = np.array(list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))))
        if points.size == 0:
            return

        # 지면(ground) 제거: z 값이 일정 임계치 이하이면 무시
        filtered_points = points[points[:,2] > self.ground_threshold]

        # 빈 2D 그리드(맵) 생성: 초기 값 0은 free, 100은 occupied
        grid = np.zeros((self.height, self.width), dtype=np.int8)

        # 각 포인트의 (x, y) 좌표를 격자 셀 인덱스로 변환
        xs = filtered_points[:, 0] - self.origin_x
        ys = filtered_points[:, 1] - self.origin_y
        cell_x = np.floor(xs / self.resolution).astype(np.int32)
        cell_y = np.floor(ys / self.resolution).astype(np.int32)

        # 격자 범위 내에 있는 점들만 선택
        valid = (cell_x >= 0) & (cell_x < self.width) & (cell_y >= 0) & (cell_y < self.height)
        cell_x = cell_x[valid]
        cell_y = cell_y[valid]

        # 해당 셀을 '점유' 상태(100)로 표시
        grid[cell_y, cell_x] = 100

        # OccupancyGrid 메시지 구성
        occ_grid = OccupancyGrid()
        occ_grid.header = msg.header
        occ_grid.header.frame_id = "map"  # 적절한 좌표계로 설정 (예: "map")
        occ_grid.info.resolution = self.resolution
        occ_grid.info.width = self.width
        occ_grid.info.height = self.height
        occ_grid.info.origin = Pose(Point(self.origin_x, self.origin_y, 0.0),
                                    Quaternion(0, 0, 0, 1))

        # OccupancyGrid 데이터는 1차원 리스트로 row-major 순서로 제공되어야 함.
        occ_grid.data = grid.flatten().tolist()

        # 결과 메시지 퍼블리시
        self.pub.publish(occ_grid)

if __name__ == "__main__":
    rospy.init_node("occupancy_grid_mapper")
    mapper = OccupancyGridMapper()
    rospy.spin()
