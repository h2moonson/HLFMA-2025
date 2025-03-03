#! /usr/bin/env python3

# path 경로를 받기 위함
import sys

# 경로 관련 모듈
from nav_msgs.msg import Path
from erp_drive.msg import PathReference
from erp_drive.msg import PathReferenceElement

# Pose 등등 위치 관련 모듈
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

# 오일러 <-> 쿼터니언 변환 관련 모듈
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from scipy.interpolate import CubicHermiteSpline


# 라이다 관련 -> 패키지명 확인 후 수정 
from std_msgs.msg import Int16
from visualization_msgs.msg import MarkerArray 
from std_msgs.msg import Float64
# from lidar_object_detection.msg import ObjectInfo

# ros 관련 모듈
import rospy
import rospkg

#사용법 : planner쪽에서 path_reader객체 생성 후 
# path_reader = pathReader('erp_drive') #경로 파일의 위치
# self.global_path, self.global_path2  = path_reader.read_txt(self.path_name+".txt")

class PathReader : 
    def __init__(self, pkg_name):
        rospack = rospkg.RosPack() 
        self.file_path = rospack.get_path(pkg_name) #erp_drive패키지
        self.path = PathReference()
        self.path2 = PathReference()
        self.rviz_g_path = Path()
        self.rviz_g_path2 = Path()
        self.start_x = 0.
        self.start_y = 0.
        
    def set_waypoints(self, file_name): 
        full_file_name = f'{self.file_path}/paths/{file_name}.txt'
        with open(full_file_name, 'r') as f:
            lines = f.readlines()
            for line in lines:
                pose = Pose()
                pose2 = Pose()
            
                x, y, yaw, mode = map(float, line.split())
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.
                pose2.position.x = x + 3 * np.cos(yaw + np.pi / 2)
                pose2.position.y = y + 3 * np.sin(yaw + np.pi / 2)
                pose2.position.z = 0.
                
                quat = quaternion_from_euler(0., 0., yaw)
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose2.orientation.x = quat[0]
                pose2.orientation.y = quat[1]
                pose2.orientation.z = quat[2]
                pose2.orientation.w = quat[3]

                element = PathReferenceElement()
                element2 = PathReferenceElement()

                element.pose = pose
                element.mode = int(mode)

                element2.pose = pose2
                element2.mode = int(mode)
                
                self.path.path.append(element)
                self.path2.path.append(element2)
         
                if self.start_x == 0 and self.start_y == 0:
                    self.start_x = pose.position.x
                    self.start_y = pose.position.y
                    
                ##### 시각화 ####
                rviz_pose = PoseStamped()
                rviz_pose.header.frame_id = 'map'
                rviz_pose.header.stamp = rospy.Time.now()
                rviz_pose2 = PoseStamped()
                rviz_pose2.header.frame_id = 'map'
                rviz_pose2.header.stamp = rospy.Time.now()
                
                rviz_pose.pose.position.x = x - self.start_x
                rviz_pose.pose.position.y = y - self.start_y
                rviz_pose.pose.position.z = 0.
                rviz_pose2.pose.position.x = pose2.position.x- self.start_x
                rviz_pose2.pose.position.y = pose2.position.y - self.start_y 
                rviz_pose2.pose.position.z = 0.
                
                rviz_pose.pose.orientation.x = quat[0]
                rviz_pose.pose.orientation.y = quat[1]
                rviz_pose.pose.orientation.z = quat[2]
                rviz_pose.pose.orientation.w = quat[3]
                rviz_pose2.pose.orientation.x = quat[0]
                rviz_pose2.pose.orientation.y = quat[1]
                rviz_pose2.pose.orientation.z = quat[2]
                rviz_pose2.pose.orientation.w = quat[3]
                
                self.rviz_g_path.poses.append(rviz_pose)
                self.rviz_g_path2.poses.append(rviz_pose2)
        is_waypoint_set = True         
        self.rviz_g_path.header.stamp = rospy.Time.now()
        self.rviz_g_path2.header.stamp = rospy.Time.now()

        rospy.loginfo(f'size of waypoint : {len(self.path.path)}')

        return is_waypoint_set, self.path, self.path2, self.rviz_g_path, self.rviz_g_path2

# class LocalPathMaker : 
#     def __init__(self) -> None:
#         # 생성자에다가 미션 argument : 소형정적/ 대형정적 주고
#         # current_pose, index 받고 나머지는 멤버 함수로 계산 
#         pass
    
def make_avoidance_path_for_narrow(curr_pose: Pose, tgt: list):
    yaw = tgt[2]

    local_path = PathReference()
    rviz_l_path = Path()
    margin = 4. # to make sure the vehicle completely escape from the narrow space

    x = np.array([0 , tgt[0][0], tgt[1][0], tgt[1][0] + margin])
    y = np.array([0 , tgt[0][1], tgt[1][1], tgt[1][1]         ]) # set 4th element of y array as tgt[1][1] to avoid from rear side hitting the obstacle 
    f = CubicHermiteSpline(x, y, dydx = [0, yaw])

    path_coeff = f.c.T 

    resolution = int(np.sqrt((tgt[1][0] + margin)**2 + tgt[1][1]**2) / 0.2) # decide the number of waypoints we'll make considering the distance to the last element  
    path_x = np.linspace(x[0], x[-1], resolution)
    path_y = f(path_x)
    path_ori = []

    for i in range(len(path_x)):
        for j in range(1, len(x)):
            if path_x[i] < x[j]:
                a, b, c, _ = path_coeff[j - 1]
                path_ori.append(3 * a * (path_x[i] - x[j - 1]) ** 2 + 2 * b * (path_x[i] - x[j - 1]) + c)
                break
        else:
            a, b, c, _ = path_coeff[-1] 
            path_ori.append(3 * a * (path_x[i] - x[5]) ** 2 + 2 * b * (path_x[i] - x[5]) + c)
            break
        
        
        local_pose = Pose()
        element = PathReferenceElement()
        element.pose = local_pose
        element.mode = 11 # self.avoidance_path_mode
        local_pose.position.x = curr_pose.position.x + path_x[i]
        local_pose.position.y = curr_pose.position.y + path_y[i]
        local_pose.position.z =  0.
        quat = quaternion_from_euler(0., 0., path_ori[i])
        local_pose.orientation.x = quat[0]
        local_pose.orientation.y = quat[1]
        local_pose.orientation.z = quat[2]
        local_pose.orientation.w = quat[3]
            
        local_path.path.append(element)

        rviz_local_pose = PoseStamped()
        rviz_local_pose.header.frame_id = 'map'
        rviz_local_pose.header.stamp = rospy.Time().now()
        
        rviz_local_pose.pose.position.x = local_pose.position.x
        rviz_local_pose.pose.position.y = local_pose.position.y
        rviz_local_pose.pose.position.z = 0.
        
        rviz_local_pose.pose.orientation.x = quat[0]
        rviz_local_pose.pose.orientation.y = quat[1]
        rviz_local_pose.pose.orientation.z = quat[2]
        rviz_local_pose.pose.orientation.w = quat[3]
        
        rviz_l_path.poses.append(rviz_local_pose)

    rviz_l_path.header.stamp = rospy.Time.now()
    is_local_waypoint_set = True 
    avoidance_in_progress = True 
    return local_path, rviz_l_path, is_local_waypoint_set, avoidance_in_progress

def make_avoidance_path(curr_pose: Pose, tgt : tuple):
    # 현재 차량 정보, tgt지점: (rel_x, rel_y, rel_yaw)
    yaw =tgt[2]

    # return path 
    local_path = PathReference() 
    rviz_l_path = Path()

    # CubicHermiteSpline
    x = np.array([0   ,      tgt[0]])
    y = np.array([0   ,      tgt[1]])
    f = CubicHermiteSpline(x, y, dydx = [0 , yaw])

    path_coeff = f.c.T #구간별 함수의 계수

    resolution = int(np.sqrt(tgt[0]**2 + tgt[1]**2) / 0.2)
    path_x = np.linspace(x[0], x[-1], resolution)
    path_y = f(path_x)
    path_ori = []
    
    for i in range(len(path_x)):
        for j in range(1, len(x)):
            if path_x[i] < x[j]:
                a, b, c, _ = path_coeff[j - 1]
                path_ori.append(3 * a * (path_x[i] - x[j - 1]) ** 2 + 2 * b * (path_x[i] - x[j - 1]) + c)
                break
        else:
            a, b, c, _ = path_coeff[-1] 
            path_ori.append(3 * a * (path_x[i] - x[5]) ** 2 + 2 * b * (path_x[i] - x[5]) + c)
            break
        
        
        local_pose = Pose()
        element = PathReferenceElement()
        element.pose = local_pose
        element.mode = 11 # self.avoidance_path_mode
        local_pose.position.x = curr_pose.position.x + path_x[i]
        local_pose.position.y = curr_pose.position.y + path_y[i]
        local_pose.position.z =  0.
        quat = quaternion_from_euler(0., 0., path_ori[i])
        local_pose.orientation.x = quat[0]
        local_pose.orientation.y = quat[1]
        local_pose.orientation.z = quat[2]
        local_pose.orientation.w = quat[3]
            
        local_path.path.append(element)

        rviz_local_pose = PoseStamped()
        rviz_local_pose.header.frame_id = 'map'
        rviz_local_pose.header.stamp = rospy.Time().now()
        
        rviz_local_pose.pose.position.x = local_pose.position.x
        rviz_local_pose.pose.position.y = local_pose.position.y
        rviz_local_pose.pose.position.z = 0.
        
        rviz_local_pose.pose.orientation.x = quat[0]
        rviz_local_pose.pose.orientation.y = quat[1]
        rviz_local_pose.pose.orientation.z = quat[2]
        rviz_local_pose.pose.orientation.w = quat[3]
        
        rviz_l_path.poses.append(rviz_local_pose)

    rviz_l_path.header.stamp = rospy.Time.now()
    is_local_waypoint_set = True 
    avoidance_in_progress = True 
    return local_path, rviz_l_path, is_local_waypoint_set, avoidance_in_progress

def calc_obst_g_coord(curr_pose : Pose, obstacle : tuple):
        # msg : (centerX, centerY, centerZ, lenX, lenY, lenZ)
        quat = curr_pose.orientation 
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        _, _, curr_yaw = euler_from_quaternion(orientation_list)

        rel_x = obstacle[0]
        rel_y = obstacle[1]
        dist = np.sqrt(rel_x**2 +  rel_y**2)
        rel_yaw  = np.arctan2(rel_y, rel_x) 
        # obstacle's global coordinate
        yaw = curr_yaw + rel_yaw
        global_x = curr_pose.position.x + dist * np.cos(yaw)
        global_y = curr_pose.position.y + dist * np.sin(yaw)
        result = (global_x, global_y) 
        return result 

def CCW(curr_pose : Pose, blocked_coord, obst_coord):
    cross_product = (blocked_coord[0] - curr_pose.position.x) * (obst_coord[1] - blocked_coord[1]) - (blocked_coord[1] - curr_pose.position.y) * (obst_coord[0] - blocked_coord[0])

    if cross_product > 0:
        return -1 # 시계 반대방향인 경우
    elif cross_product < 0:
        return 1  # 시계방향인 경우
    else:
        return 0
    

def is_obst_on_path (ego_status : Pose, curr_idx : int, path : PathReference, local_path_ld : float, g_coord : tuple):
    # start ~ end 까지 경로 상의 탐색 영역을 정해서 해당 path와 장애물이 겹치면 경로를 막는다고 판단
    roi = path # region of interest
    roi_ld = local_path_ld # works as a sensitivity 
    obst_radius = 2. # works as an inflation radius of 
    start = curr_idx
    end = min(curr_idx + (roi_ld // 0.2), len(roi.path))
    for i in range (start, end) : 
        x = roi.path[i].pose.position.x 
        y = roi.path[i].pose.position.y
        dx = x - g_coord[0]
        dy = y - g_coord[1]
        dist = np.sqrt(dx**2 + dy**2)
        if dist <= obst_radius : 
            blocked_idx = i #해당 인덱스 위를 장애물이 가로막고 있는 것 
            #이때 CCW에 들어가게 되는 min_obst_coord이 결정됨 
            blocked_coord = (x, y)
            # obst = (g_coord[0], g_coord[1])
            # direction = CCW(ego_status, blocked_coord, obst)
            return True, blocked_idx, blocked_coord
    return False, None, None