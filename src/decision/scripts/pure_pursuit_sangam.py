#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Int64MultiArray, Float64, Int64, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import  Vector3
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import GPSMessage, CtrlCmd, EventInfo, EgoVehicleStatus
from morai_msgs.srv import MoraiEventCmdSrv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils_sangam import pathReader,findCurrentwaypoint,findLocalPath,purePursuit,rotateLiDAR2GPS, CCW, pidController
from lidar_object_detection.msg import ObjectInfo
from lidar_cam_fusion.msg import Float64Array2D
from ultralytics_ros.msg import YoloResult


import tf
from math import *
import numpy as np
from tabulate import tabulate
import os
import time 

# 아이오닉 5 -> 조향값(steering_msg) 0일 때 직진 양수이면 좌회전 음수이면 우회전

class EgoStatus:
    def __init__(self):
        self.position = Vector3()
        self.heading = 0.0
        self.velocity = Vector3()


class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        self.path_name = 'mando_path'
        # self.path_name = 'tmp'
        self.remove_path = 'remove_path'
        self.remove_path_2 = 'remove_path_2'
        self.left_path = 'left_path'
        self.center_path = 'center_path'
        self.right_path = 'right_path'
        self.left_1_path = 'left_1'
        self.center_1_path = 'center_1'
        self.right_1_path = 'right_1'
        self.current_waypoint = 0

        # Lattice Planner Parameters
        lattice_lane_cnt = 4
        global_lattice_lane = 3
  
        self.lattice_lidar_obstacle_info = []
        self.obstacle_info_removed = []
        self.lattice_distance_threshold = 0.0
        self.selected_lane = 0
        self.pathswitching_lane = 2
        self.is_lattice_working = False
        


        # Publisher
        self.global_path_pub                = rospy.Publisher('/global_path', Path, queue_size=1) ## global_path publisher 
        self.local_path_pub                 = rospy.Publisher('/local_path', Path, queue_size=1)
        self.remove_path_pub                = rospy.Publisher('remove_path', Path, queue_size=1)
        self.remove_path_2_pub              = rospy.Publisher('remove_path_2', Path, queue_size=1)
        self.left_path_pub              = rospy.Publisher('left_path', Path, queue_size=1)
        self.center_path_pub              = rospy.Publisher('center_path', Path, queue_size=1)
        self.right_path_pub              = rospy.Publisher('right_path', Path, queue_size=1)
        self.left_path_1_pub              = rospy.Publisher('left_1', Path, queue_size=1)
        self.center_path_1_pub              = rospy.Publisher('center_1', Path, queue_size=1)
        self.right_path_1_pub              = rospy.Publisher('right_1', Path, queue_size=1)
        self.ctrl_cmd_pub                   = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.lattice_obstacle_pub           = rospy.Publisher('/lattice_obstacle_marker_array', MarkerArray, queue_size=1)
        self.acc_obstacle_pub               = rospy.Publisher('/acc_obstacle_marker_array', MarkerArray, queue_size=1)
        self.pure_pursuit_target_point_pub  = rospy.Publisher('/pure_pusuit_target_point', Marker, queue_size=1)
        self.curvature_target_point_pub     = rospy.Publisher('/curvature_target_point', Marker, queue_size=1)
        self.ego_marker_pub                 = rospy.Publisher('/ego_marker', Marker, queue_size=1)
        
        
        ########################  lattice  ########################
        for i in range(1,lattice_lane_cnt+1):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  

        for i in range(1,lattice_lane_cnt+1):            
            globals()['planning_lattice_path_{}_pub'.format(i)]=rospy.Publisher('planning_lattice_path_{}'.format(i),Path,queue_size=1)  
        ########################  lattice  ########################

        # Subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/imu", Imu, self.imuCB) ## Vehicle Status Subscriber
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.egoStatusCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/object_info", ObjectInfo, self.lidarObjectCB)
        rospy.Subscriber("/traffic_light", Int64MultiArray, self.trafficlightCB)
        # rospy.Subscriber("/object_info_acc", ObjectInfo, self.accLidarObjectCB)
        rospy.Subscriber("/yolo_result", YoloResult, self.yoloResultCB)
        # rospy.Subscriber("/object_info_dynamic", ObjectInfo, self.dynamicLidarObjectCB)
        # rospy.Subscriber("/fusion_result", Float64Array2D, self.fusionResultCB)
        
        self.status_msg   = EgoStatus()
        self.ctrl_cmd_msg = CtrlCmd()

        self.is_lab_time_check_started = False
        self.is_lab_time_check_finished = False

        ### 동적 장애물 미션 파라미터 ###
        # self.clock_wise = 0
        # self.dynamic_obstacle_info = []
        # self.dynamic_obstacle_distance_threshold = 0.0
        # self.min_distance = 99999
        # self.min_path_coord = []
        # self.min_obstacle_coord = []
        # self.perception_not_working = 0
        # self.dynamic_obstacle_coord_finish = ()
        # self.dynamic_obstacle_coord_start = ()
        # self.dynamic_obstacle_continue = False
        # self.should_not_update_fusion_result = True 
        ###############################


        self.is_status = False
        self.is_gps = False
        self.is_imu = False
        self.euler_data = [0,0,0,0]
        self.quaternion_data = [0,0,0,0]

        self.steering_angle_to_servo_offset = 0.0 ## servo moter offset
        self.target_x = 0.0
        self.target_y = 0.0
        self.curvature_target_x = 0.0
        self.curvature_target_y = 0.0
        self.corner_theta_degree = 0.0

        self.motor_msg = 0.0
        self.steering_msg = 0.0
        self.brake_msg = 0.0

        self.steering_offset = 0.015
        # self.steering_offset = 0.05 

        self.lfd = 0.0
        self.min_distance_from_path = 0.0
        self.distance_from_remove_path = 0.0
        
        self.curve_steering_msg = 0.0
        self.curve_motor_msg = 0.0

        ########traffic_stop_#######
        self.green_light_count = 0
        self.red_light_count = 0

        self.stopline_flag = False
        self.current_waypoint = 0
        self.mission_name = "default"

        self.proj_UTM = Proj(proj='utm', zone = 52, elips='WGS84', preserve_units=False)
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.max_velocity = 50

        self.current_velocity = 0
        self.current_wheel_angele= 0
        self.brake_cnt = 0
        self.path_switching_num = 0

        self.traffic_stop_index_1 = 545
        self.traffic_stop_index_2 = 1535

        self.center_distance_threshold = 0.0
        self.angle_distance_threshold = 0.0

        self.lidar_obstacle_info = []
        self.previous_obstacle_info_removed = []
        self.obstacle_info_removed = []
        self.obstacle_info_lattice = []
        self.yolo_bbox_size_list = []
        self.yolo_result_list = []
        self.fusion_result_person = []
        self.collidable_obstacle_info = []
        self.obstacle_movement_list = []
        self.collision_bool = [False]
        
        self.lattice_start_position_x = 0.0
        self.lattice_start_position_y = 0.0
        self.distance_start2current = 0.0
        self.is_acc_working = True
        self.is_path_switching = False
        self.path_switching_cycle_start = False

        pid=pidController()   

        ######################################## For Service ########################################
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.req_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.req = EventInfo()
        #############################################################################################

        # Class
        path_reader = pathReader('path_maker') ## 경로 파일의 위치
        self.pure_pursuit = purePursuit() ## purePursuit import

        # Read path
        self.global_path = path_reader.read_txt(self.path_name+".txt")
        self.mando_path = path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름
        self.remove_path = path_reader.read_txt(self.remove_path+".txt")
        self.remove_path_2 = path_reader.read_txt(self.remove_path_2 +".txt")
        self.left_path = path_reader.read_txt(self.left_path+".txt")
        self.center_path = path_reader.read_txt(self.center_path+".txt")
        self.right_path = path_reader.read_txt(self.right_path+".txt")
        self.left_1_path = path_reader.read_txt(self.left_1_path+".txt")
        self.center_1_path = path_reader.read_txt(self.center_1_path+".txt")
        self.right_1_path = path_reader.read_txt(self.right_1_path+".txt")

        rate = rospy.Rate(40) 
                                           
        while not rospy.is_shutdown():

            self.getEgoStatus()
            # print(self.current_waypoint)
            # Lattice Planner Parameters
            if 1212 <= self.current_waypoint <= 1426:
                lattice_lane_cnt = 3
                global_lattice_lane = 2
            elif 1427 <= self.current_waypoint <= 1547:
                lattice_lane_cnt = 4
                global_lattice_lane = 2
            elif 1548 <= self.current_waypoint <= 1648:
                lattice_lane_cnt = 3
                global_lattice_lane = 1
            elif 1649 <= self.current_waypoint <= 1799:
                lattice_lane_cnt = 3
                global_lattice_lane = 2
            # elif 1800 <= self.current_waypoint <= 2024:
            #     lattice_lane_cnt = 2
            #     global_lattice_lane = 1
           
           
           
            elif 1800 <= self.current_waypoint <= 2019:
                lattice_lane_cnt = 2
                global_lattice_lane = 1

            elif 2020 <= self.current_waypoint <= 2057:
                lattice_lane_cnt = 4
                global_lattice_lane = 3

            # elif 2043 <= self.current_waypoint <= 2052:
            #     lattice_lane_cnt = 3
            #     global_lattice_lane = 1
            # elif 2025 <= self.current_waypoint <= 2035:
            #     lattice_lane_cnt = 3
            #     global_lattice_lane = 1
            # elif 2036 <= self.current_waypoint <= 2054:
            #     lattice_lane_cnt = 3
            #     global_lattice_lane = 1  
            elif 2058 <= self.current_waypoint <= 2072:
                lattice_lane_cnt = 3
                global_lattice_lane = 2     
           
           
           
           
           
           
           
           
           
           
            elif 2073 <= self.current_waypoint <= 2410:
                lattice_lane_cnt = 3
                global_lattice_lane = 1 
            else:
                lattice_lane_cnt = 4
                global_lattice_lane = 3

            if (self.current_waypoint < 1590) or (1670 < self.current_waypoint < 2000) or (2090 < self.current_waypoint): 
                self.is_path_switching = False
                self.global_path = self.mando_path
                # self.mission_name = "acc"
            
            if self.is_status == True:
                
                self.mission_name = "acc"
                
                self.ctrl_cmd_msg.longlCmdType = 1
                self.current_waypoint = findCurrentwaypoint(self.mando_path,self.status_msg)
                local_path, self.min_distance_from_path = findLocalPath(self.global_path, self.status_msg)
               
                ######################## LiDAR 좌표계에서 Detect 된 장애물 GPS 좌표계로 변환하는 구간 ########################
                # self.obstacle_info_removed = rotateLiDAR2GPS(self.lattice_lidar_obstacle_info, self.status_msg, self.current_waypoint, self.selected_lane)

                self.lidar_obstacle_info.sort()
                self.obstacle_info_rotated = rotateLiDAR2GPS(self.lidar_obstacle_info, self.status_msg, self.current_waypoint)
                
                # self.obstacle_info_removed = self.pure_pursuit.isObstacleOnRemovePath(self.remove_path, self.obstacle_info_rotated, self.current_waypoint)
                if 800<= self.current_waypoint <= 1070:

                    self.obstacle_info_removed = self.pure_pursuit.isObstacleOnRemovePath(self.remove_path, self.obstacle_info_rotated, self.current_waypoint)
                    self.obstacle_info_removed = self.pure_pursuit.isObstacleOnLeftRemovePath(self.remove_path_2, self.obstacle_info_removed, self.current_waypoint)
                
                else:
                    self.obstacle_info_removed = self.pure_pursuit.isObstacleOnRemovePath(self.remove_path, self.obstacle_info_rotated, self.current_waypoint)
                #####################################################################################################
               
                if self.is_path_switching:
                    # print("path switching 시작")
                    # if self.path_switching_cycle_start:
                    #     print("calculating path_switching num")
                    if (1590 < self.current_waypoint <= 1670):
                        self.path_switching_num = self.pure_pursuit.PathSwitchingPlanner(self.left_1_path,self.center_1_path,self.right_1_path, self.obstacle_info_removed, self.status_msg, self.pathswitching_lane)
                        if self.path_switching_num == 0:
                            self.global_path = self.left_1_path
                            # print("11111111")
                        
                        elif self.path_switching_num == 1:
                            self.global_path = self.center_1_path
                            # print("22222222")
                        
                        elif self.path_switching_num == 2:
                            self.global_path = self.right_1_path
                            # print("333333333")
                    
                    
                    if (2000<= self.current_waypoint <= 2090):
                        print("path_switching 두 번째 구간")
                        self.path_switching_num = self.pure_pursuit.PathSwitchingPlanner(self.left_path,self.center_path,self.right_path, self.obstacle_info_removed, self.status_msg, self.pathswitching_lane)

                        if self.path_switching_num == 0:
                            self.global_path = self.left_path
                            # print("11111111")
                        
                        elif self.path_switching_num == 1:
                            self.global_path = self.center_path
                            # print("22222222")
                        
                        elif self.path_switching_num == 2:
                            self.global_path = self.right_path
                            # print("333333333")
                    # else: 
                    #     print("not calculating")


                if (455 <= self.current_waypoint <= 555) or (1425 <= self.current_waypoint <= 1545):
                    # print("max vel is 20")
                    self.max_velocity = 20
                else:
                    if any(object[0] >= 400.0 and object[1] == 0 for object in self.yolo_result_list):
                        # print("PPPPPPPPEEEEEEDDDRRREEUUUMMM")
                        self.max_velocity = 30
                    else:
                        self.max_velocity = 50
                

                # print(self.current_waypoint)
                # print(self.is_lattice_working)


                # lattice_path, planning_lattice_path, self.selected_lane, self.lattice_distance_threshold, self.collision_bool, self.obstacle_info_lattice = self.pure_pursuit.latticePlanner(local_path, self.obstacle_info_removed, self.status_msg, global_lattice_lane, self.current_waypoint)
                if not self.is_acc_working or self.is_lattice_working:
                    

                    # 코너에서는 Lattice 끄기  하지만 끄게 되면 코너에서 나오는 동적(인간) 못 피하게 됨
                    if (not self.is_acc_working):

                        self.mission_name = "lattice"
                        # self.brake_cnt = 0

                        dx = self.status_msg.position.x - self.lattice_start_position_x
                        dy = self.status_msg.position.y - self.lattice_start_position_y

                        self.distance_start2current = sqrt(dx**2 + dy**2)

                        # if self.distance_start2current < 1.5:
                        #     self.obstacle_info_removed = self.previous_obstacle_info_removed
                        lattice_path, planning_lattice_path, self.selected_lane, self.lattice_distance_threshold, self.collision_bool, self.obstacle_info_lattice = self.pure_pursuit.latticePlanner(local_path, self.obstacle_info_removed, self.status_msg, global_lattice_lane, self.current_waypoint)
                        self.visualizeLatticeObstacle()

                        if len(lattice_path)==lattice_lane_cnt:                    
                            for i in range(1,lattice_lane_cnt+1):
                                globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                                globals()['planning_lattice_path_{}_pub'.format(i)].publish(planning_lattice_path[i-1])


                        if all(self.collision_bool): 
                            self.brake() 
                            # print("!!!!!!!!!") 
                            continue              
                            # self.mission_name = "acc"
                            # print("################# all collision ##########################")
                            # self.is_acc_working = True
                            # self.is_lattice_working = False


                        if self.selected_lane != -1: 
                            local_path = lattice_path[self.selected_lane]                
                        

                        if global_lattice_lane != self.selected_lane: 
                            self.is_lattice_working = True
                            # self.brake_cnt = 0

                        elif self.min_distance_from_path <= 3.0:   # 2.0
                           
                            if self.distance_start2current > 6.0:
                                
                                self.mission_name = "acc"
                                self.is_lattice_working = False 

                        else:
                            pass
                   
                else:
                    lattice_path = []
                    planning_lattice_path = []

                    for i in range(lattice_lane_cnt):
                        tmp = Path()
                        tmp.header.frame_id='map'
                        lattice_path.append(tmp)
                        planning_lattice_path.append(tmp)

                    if len(lattice_path)==lattice_lane_cnt: 

                        for i in range(1,lattice_lane_cnt+1):
                            globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                            globals()['planning_lattice_path_{}_pub'.format(i)].publish(planning_lattice_path[i-1])
                        
       
                self.pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                self.pure_pursuit.getEgoStatus(self.status_msg) 

               
                # print("self.distance_from_remove_path",self.distance_from_remove_path)
                # print(len(self.obstacle_info_removed))
                is_obstacle_on_path, distance_object_to_car_list, self.center_distance_threshold, self.angle_distance_threshold, self.collidable_obstacle_info = self.pure_pursuit.isObstacleOnPath(local_path, self.obstacle_info_removed, self.status_msg, self.current_waypoint,self.is_path_switching) 
                # print(len(self.collidable_obstacle_info))
                self.collidable_obstacle_info.sort(key=lambda x:x[-2]) # 기존에는 그냥 sort() 였는데, ENU 좌표계 기준이다 보니 차량에 가장 가까운 장애물의 x좌표가 무조건 작다는 보장 없음. 따라서 차와의 실제 거리로 sort 때림

                # print(is_obstacle_on_path, self.center_distance_threshold)
                
                if not self.is_path_switching:
                    self.steering, self.target_x, self.target_y, self.lfd = self.pure_pursuit.steeringAngle(False, 0, self.mission_name, self.min_distance_from_path)
                
                else:
                    # self.mission_name = "pathswitching"
                    if self.min_distance_from_path < 1.5: 
                        print("path_switching : < 1.5")
                        self.steering, self.target_x, self.target_y, self.lfd = self.pure_pursuit.steeringAngle(False, 0, self.mission_name, self.min_distance_from_path)
                        # self.path_switching_cycle_start = True
                        
                    else : 
                        print("path_switching : > 1.5")
                        self.steering, self.target_x, self.target_y, self.lfd = self.pure_pursuit.steeringAngle(True, 0, self.mission_name, self.min_distance_from_path)
                        self.max_velocity = 30
                        # self.path_switching_cycle_start = False

                self.corner_theta_degree, self.curvature_target_x, self.curvature_target_y = self.pure_pursuit.estimateCurvature()
                
                self.target_velocity = self.cornerController(self.max_velocity, self.corner_theta_degree)

                control_input = pid.pid(self.target_velocity, self.current_velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)
                
                if control_input > 0 :
                    self.accel_msg= control_input
                    self.brake_msg = 0
                else :
                    self.accel_msg = 0
                    self.brake_msg = -control_input

                self.steering_msg = (self.steering + 2.7) * self.steering_offset

               
                # if self.current_waypoint <= 670 or 1457 <= self.current_waypoint < 2542:
                #     self.mission_name = "ACC"

                # try:
                if self.mission_name == "acc" and len(distance_object_to_car_list) != 0: 

                    distance_object_to_car_list.sort()
                    distance_object_to_car = distance_object_to_car_list[0]

                    if is_obstacle_on_path and distance_object_to_car < 25:

                        # control_input = pid.pid(self.target_velocity, self.current_velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)
    
                        self.accel_msg = 0
                        self.brake_msg = 1

                # except:
                #     pass
                
                
                if self.mission_name == "acc":     

                          
                    self.lattice_start_position_x = self.status_msg.position.x
                    self.lattice_start_position_y = self.status_msg.position.y
                    self.distance_start2current = 0.0
                    self.is_acc_working = True
                    
                    if self.brake_msg == 1:
                        self.brake_cnt += 1
                        # if len(self.obstacle_movement_list) <= 150:
                        self.obstacle_movement_list.append(self.collidable_obstacle_info[0])
                       
                    else:
                        self.brake_cnt -= 5 # 10
                    
                    if self.brake_cnt < 0:
                        self.brake_cnt = 0
                        self.obstacle_movement_list = []
                    
                    
                    
                    if self.brake_cnt >= 120:       # 150

                        self.lattice_start_position_x = self.status_msg.position.x
                        self.lattice_start_position_y = self.status_msg.position.y

                        # obstacle_movement = self.obstacle_movement_list[-1] - self.obstacle_movement_list[50]

                        if len(self.obstacle_movement_list) >= 120:     # 150
                            obstacle_movement = sqrt(pow(self.obstacle_movement_list[-1][0]-self.obstacle_movement_list[100][0],2)+pow(self.obstacle_movement_list[-1][1]-self.obstacle_movement_list[100][1],2))
                        else:
                            obstacle_movement = float('inf')

                        if obstacle_movement > 0.5: # 동적으로 판단  !!!!!!!!!안되면 0.5로 다시 바꾸세요  1.0
                            self.brake_cnt = 0
                            self.obstacle_movement_list = []
            

                        else : # 정적으로 판단 
                            if (1590 < self.current_waypoint <= 1670) or (2000<= self.current_waypoint <= 2090):
                                print("line 521 path switching 구간임")
                                self.is_acc_working =True
                                self.is_path_switching = True
                            else:
                                self.is_acc_working = False
                                self.brake_cnt = 0
                                self.obstacle_movement_list = []

                                self.previous_obstacle_info_removed = self.obstacle_info_removed
                            

                    # if self.brake_cnt >= 300:
                    #     self.brake_cnt = 0   
                    #     self.is_acc_working = False 
                    #     print("obstacle_movement",obstacle_movement)
                
                # print("brake_cnt is ", self.brake_cnt)

                log = [
                {
                    "Mission": self.mission_name, 
                    "Waypoint": self.current_waypoint, 
                    "BrakeCnt": self.brake_cnt,
                    "Distance": self.distance_start2current,
                    # "Red": self.red_light_count,
                    # "Green": self.green_light_count,
                    # "targetvelocity": self.target_velocity
                }
                ]
                os.system("clear")
                print(tabulate(log, headers="keys", tablefmt="grid"))


                if self.isTrafficLightArea() and self.isRedLight():
                    self.brake_cnt = 0
                    self.brake()
                    continue

                self.visualizeTargetPoint()
                self.visualizeCurvatureTargetPoint()
                self.visualizeEgoMarker()

                # self.visualizeLatticeObstacle()
                self.visualizeAccObstacle()
                # self.visualizeDynamicObstacle()
                
                
                self.local_path_pub.publish(local_path)
                self.global_path_pub.publish(self.global_path)
                self.remove_path_pub.publish(self.remove_path)
                self.remove_path_2_pub.publish(self.remove_path_2)
                self.left_path_pub.publish(self.left_path)
                self.center_path_pub.publish(self.center_path)
                self.right_path_pub.publish(self.right_path)
                self.left_path_1_pub.publish(self.left_1_path)
                self.center_path_1_pub.publish(self.center_1_path)
                self.right_path_1_pub.publish(self.right_1_path)

                ####################### 종료 정지선 브레이크 #######################
                if self.current_waypoint >= 2415:
                    self.mission_name = "The End"
                    self.brake()
                    self.parking()
                    continue
                ################################################################
              
                ########################################################################################################################################################
                self.publishCtrlCmd(self.accel_msg, self.steering_msg, self.brake_msg)
                # print("current_waypoint: ", self.current_waypoint)
                ########################################################################################################################################################

            else:
              
                self.publishCtrlCmd(self.motor_msg, self.steering_msg, self.brake_msg)
            # print("mission",self.mission_name)
            ##########################################printlog#########################################
            

            rate.sleep()
    
###################################################################### Service Request  ######################################################################
    # option - 1 : ctrl_mode / 2 : gear / 4 : lamps / 6 : gear + lamps
    # gear - 1: P / 2 : R / 3 : N / 4 : D
##############################################################################################################################################################

    def forward_mode(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 0
        self.req.lamps.emergencySignal = 0
        response = self.req_service(self.req)
        self.yaw_rear = False

    def rear_mode(self):
        self.req.option = 2
        self.req.gear = 2
        response = self.req_service(self.req)
        self.yaw_rear = True

    def drive_left_signal(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 1
        response = self.req_service(self.req)

    def drive_right_signal(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 2
        response = self.req_service(self.req)

    def emergency_mode(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.emergencySignal = 1
        response = self.req_service(self.req)

    def parking(self) :
        self.req.option = 6
        self.req.gear = 1
        self.req.lamps.turnSignal = 0
        response = self.req_service(self.req)

    def brake(self) :
        self.ctrl_cmd_msg.longlCmdType = 1 # log = [
                # {
                #     "Mission": self.mission_name, 
                #     "Waypoint": self.current_waypoint, 
                #     "BrakeCnt": self.brake_cnt,
                #     "Distance": self.distance_start2current,
                #     "Red": self.red_light_count,
                #     "Green": self.green_light_count,
                #     "targetvelocity": self.target_velocity
                # }
                # ]
                # os.system("clear")
                # print(tabulate(log, headers="keys", tablefmt="grid"))
        self.accel_msg = 0.0
        self.steering_msg = 0.0
        self.brake_msg = 1.0
        self.publishCtrlCmd(self.accel_msg, self.steering_msg, self.brake_msg)
    
###################################################################### Call Back ######################################################################

    def getEgoStatus(self): ## Vehicle Status Subscriber 
        if self.is_gps == True and self.is_imu == True:
            self.status_msg.position.x = self.xy_zone[0] - 313008.55819800857
            self.status_msg.position.y = self.xy_zone[1] - 4161698.628368007
            self.status_msg.position.z = 0.0
            self.status_msg.heading = self.euler_data[2] * 180/pi
            self.status_msg.velocity.x = self.current_velocity

            self.tf_broadcaster.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                            tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                            rospy.Time.now(),
                            "base_link",
                            "map")
   
            self.is_status=True

        elif self.is_gps is False and self.is_imu is True:
            self.status_msg.heading = self.euler_data[2] * 180/pi
            self.is_status=False

        else:
            # print("Waiting for GPS & IMU")
            self.is_status=False

    def gpsCB(self, msg):
        self.xy_zone = self.proj_UTM(msg.longitude, msg.latitude)
        
        self.tf_broadcaster.sendTransform((0, 0.1, 1.18),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "gps",
                        "base_link")
        
        self.tf_broadcaster.sendTransform((4.20, 0, 0.2),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "velodyne",
                        "base_link")
        
        self.is_gps = True

    def imuCB(self, msg):
        self.quaternion_data = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.euler_data = tf.transformations.euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))

        self.tf_broadcaster.sendTransform((-0.08, 0.0, 1.18),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "imu",
                        "base_link")

        self.is_imu = True
    
    def yoloResultCB(self, msg):
        detections_list = msg.detections.detections
        self.yolo_result_list = [[0.0, 0] for _ in range(len(detections_list))]
        
        for i in range(len(detections_list)):
            self.yolo_result_list[i] = (detections_list[i].bbox.size_x * detections_list[i].bbox.size_y, detections_list[i].results[0].id) 
            # print("size",detections_list[i].bbox.size_x * detections_list[i].bbox.size_y)


    def egoStatusCB(self, msg):
        self.current_velocity = msg.velocity.x * 3.6
        if self.current_velocity < 0 :
            self.current_velocity = 0
        self.current_wheel_angle = msg.wheel_angle

        
    def trafficlightCB(self, msg):
        self.red_light_count   = msg.data[0]
        self.green_light_count = msg.data[1]
        # print("Green:", self.green_light_count, 'Red:', self.red_light_count)


    def lidarObjectCB(self, msg):
        self.lidar_obstacle_info = [[] for i in range(msg.objectCounts)]

        for i in range(msg.objectCounts):
            # x_min = msg.centerX[i] - msg.lengthX[i] / 2
            # x_max = msg.centerX[i] + msg.lengthX[i] / 2
            # y_min = msg.centerY[i] - msg.lengthY[i] / 2
            # y_max = msg.centerY[i] + msg.lengthY[i] / 2

            # self.lidar_obstacle_info[i] = [msg.centerX[i], msg.centerY[i], msg.centerZ[i], msg.lengthX[i], msg.lengthY[i], msg.lengthZ[i], x_min, x_max, y_min, y_max]

            left_top_x = msg.centerX[i] + msg.lengthX[i]/2
            left_top_y = msg.centerY[i] + msg.lengthY[i]/2

            left_bottom_x = msg.centerX[i] - msg.lengthX[i]/2
            left_bottom_y = msg.centerY[i] + msg.lengthY[i]/2

            right_top_x = msg.centerX[i] + msg.lengthX[i]/2
            right_top_y = msg.centerY[i] - msg.lengthY[i]/2

            right_bottom_x = msg.centerX[i] - msg.lengthX[i]/2
            right_bottom_y = msg.centerY[i] - msg.lengthY[i]/2

            self.lidar_obstacle_info[i] = [msg.centerX[i], msg.centerY[i], msg.centerZ[i], 
                                           msg.lengthX[i], msg.lengthY[i], msg.lengthZ[i],
                                           left_top_x, left_top_y, # 6 7
                                           left_bottom_x, left_bottom_y, # 8 9
                                           right_top_x, right_top_y, # 10 11
                                           right_bottom_x, right_bottom_y] # 12 13

        # print(self.lidar_obstacle_info)




    # def lidarObjectCB(self, msg):
    #     self.lattice_lidar_obstacle_info = [[] for i in range(msg.objectCounts)]
    #     for i in range(msg.objectCounts):
    #         self.lattice_lidar_obstacle_info[i] = [msg.centerX[i], msg.centerY[i], msg.centerZ[i], msg.lengthX[i], msg.lengthY[i], msg.lengthZ[i]]
        # print(self.lattice_lidar_obstacle_info)

    # def dynamicLidarObjectCB(self,msg):
    #     self.dynamic_obstacle_info = [[] for i in range(msg.objectCounts)]
    #     for i in range(msg.objectCounts):
    #         self.dynamic_obstacle_info[i] = [msg.centerX[i], msg.centerY[i], msg.centerZ[i], msg.lengthX[i], msg.lengthY[i], msg.lengthZ[i]]

    # def fusionResultCB(self, msg):
    #     self.fusion_result_person = [list(bbox.bbox)[4:9] for bbox in msg.bboxes if bbox.bbox[8] == 0]
    #     self.fusion_result_person.sort()
        
        # if (len(self.fusion_result_person) == 0): 
        #     self.is_dynamic_obstacle = False
        # else: 
        #     self.is_dynamic_obstacle = True 
        #     print(self.fusion_result_person)




###################################################################### Function ######################################################################

    def publishCtrlCmd(self, accel_msg, steering_msg, brake_msg):
        self.ctrl_cmd_msg.accel = accel_msg
        self.ctrl_cmd_msg.steering = steering_msg
        self.ctrl_cmd_msg.brake = brake_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def setBrakeMsgWithNum(self, brake):
        self.brake_msg = brake
    
    def cornerController(self, max_velocity, corner_theta_degree):
        if corner_theta_degree > 40:
            corner_theta_degree = 40

        target_velocity = 40

        if max_velocity == 50:
            target_velocity = -0.52 * corner_theta_degree + 40
            # target_velocity = -0.4 * corner_theta_degree + 35

        elif max_velocity == 30:
            target_velocity = -0.3 * corner_theta_degree + 20

        else:
            # target_velocity = -0.3 * corner_theta_degree + 20
            target_velocity = max_velocity

        # 음수 예외처리
        if target_velocity < 0:
            target_velocity = 15
        
        return target_velocity 



    def visualizeLatticeObstacle(self):
        lattice_obstacle_array = MarkerArray()
     
        for i in range(len(self.obstacle_info_lattice)):

            lattice_obstacle = Marker()
            lattice_obstacle.header.frame_id = "map"
            lattice_obstacle.id = i * 321
            lattice_obstacle.type = lattice_obstacle.CYLINDER
            lattice_obstacle.action = lattice_obstacle.ADD
            lattice_obstacle.scale.x = self.obstacle_info_lattice[i][-1] * 2
            lattice_obstacle.scale.y = self.obstacle_info_lattice[i][-1] * 2
            lattice_obstacle.scale.z = 0.5
            lattice_obstacle.pose.orientation.x = 0.0
            lattice_obstacle.pose.orientation.y = 0.0
            lattice_obstacle.pose.orientation.z = 0.0
            lattice_obstacle.pose.orientation.w = 1.0
            lattice_obstacle.color.r = 1.0
            lattice_obstacle.color.g = 0.0
            lattice_obstacle.color.b = 0.0
            lattice_obstacle.color.a = 0.5 
            lattice_obstacle.pose.position.x = self.obstacle_info_lattice[i][0]
            lattice_obstacle.pose.position.y = self.obstacle_info_lattice[i][1]
            lattice_obstacle.pose.position.z = 0.0
            lattice_obstacle.lifetime = rospy.Duration(0.1)

            lattice_obstacle_array.markers.append(lattice_obstacle)

        self.lattice_obstacle_pub.publish(lattice_obstacle_array)

    def visualizeDynamicObstacle(self):
        dynamic_obstacle_array = MarkerArray()

        for i in range(len(self.dynamic_obstacle_info)):
            dynamic_obstacle = Marker()
            dynamic_obstacle.header.frame_id = "map"
            dynamic_obstacle.id = i
            dynamic_obstacle.type = dynamic_obstacle.CYLINDER
            dynamic_obstacle.action = dynamic_obstacle.ADD
            dynamic_obstacle.scale.x = self.dynamic_obstacle_distance_threshold * 2
            dynamic_obstacle.scale.y = self.dynamic_obstacle_distance_threshold * 2
            dynamic_obstacle.scale.z = 2.0
            dynamic_obstacle.pose.orientation.w = 1.0
            dynamic_obstacle.color.r = 0.0
            dynamic_obstacle.color.g = 1.0
            dynamic_obstacle.color.b = 0.0
            dynamic_obstacle.color.a = 0.5 
            dynamic_obstacle.pose.position.x = self.dynamic_obstacle_info[i][0]
            dynamic_obstacle.pose.position.y = self.dynamic_obstacle_info[i][1]
            dynamic_obstacle.pose.position.z = 0.0
            dynamic_obstacle.lifetime = rospy.Duration(0.1)

            dynamic_obstacle_array.markers.append(dynamic_obstacle)

        self.dynamic_obstacle_pub.publish(dynamic_obstacle_array)

    # def visualizeAccObstacle(self):
    #     acc_obstacle_array = MarkerArray()

    #     for i in range(len(self.collidable_obstacle_info)):
    #         acc_obstacle = Marker()
    #         acc_obstacle.header.frame_id = "map"
    #         acc_obstacle.id = i * 3  # 고유 ID 설정
    #         acc_obstacle.type = acc_obstacle.CYLINDER
    #         acc_obstacle.action = acc_obstacle.ADD
    #         acc_obstacle.scale.x = self.center_distance_threshold * 2
    #         acc_obstacle.scale.y = self.center_distance_threshold * 2
    #         acc_obstacle.scale.z = 0.5
    #         acc_obstacle.pose.orientation.x = 0.0
    #         acc_obstacle.pose.orientation.y = 0.0
    #         acc_obstacle.pose.orientation.z = 0.0
    #         acc_obstacle.pose.orientation.w = 1.0
    #         acc_obstacle.color.r = 0.0
    #         acc_obstacle.color.g = 0.0
    #         acc_obstacle.color.b = 1.0
    #         acc_obstacle.color.a = 0.5
    #         acc_obstacle.pose.position.x = self.collidable_obstacle_info[i][0]
    #         acc_obstacle.pose.position.y = self.collidable_obstacle_info[i][1]
    #         acc_obstacle.pose.position.z = 0.0


    #         acc_obstacle2 = Marker()
    #         acc_obstacle2.header.frame_id = "map"
    #         acc_obstacle2.id = i * 3 + 1  # 고유 ID 설정
    #         acc_obstacle2.type = acc_obstacle2.CYLINDER
    #         acc_obstacle2.action = acc_obstacle2.ADD
    #         acc_obstacle2.scale.x = self.center_distance_threshold * 2
    #         acc_obstacle2.scale.y = self.center_distance_threshold * 2
    #         acc_obstacle2.scale.z = 0.5
    #         acc_obstacle2.pose.orientation.x = 0.0
    #         acc_obstacle2.pose.orientation.y = 0.0
    #         acc_obstacle2.pose.orientation.z = 0.0
    #         acc_obstacle2.pose.orientation.w = 1.0
    #         acc_obstacle2.color.r = 0.0
    #         acc_obstacle2.color.g = 1.0
    #         acc_obstacle2.color.b = 0.0
    #         acc_obstacle2.color.a = 0.5
    #         acc_obstacle2.pose.position.x = self.collidable_obstacle_info[i][6]
    #         acc_obstacle2.pose.position.y = self.collidable_obstacle_info[i][7]
    #         acc_obstacle2.pose.position.z = 0.0

            

    #         acc_obstacle3 = Marker()
    #         acc_obstacle3.header.frame_id = "map"
    #         acc_obstacle3.id = i * 3 + 2  # 고유 ID 설정
    #         acc_obstacle3.type = acc_obstacle3.CYLINDER
    #         acc_obstacle3.action = acc_obstacle3.ADD
    #         acc_obstacle3.scale.x = self.center_distance_threshold * 2
    #         acc_obstacle3.scale.y = self.center_distance_threshold * 2
    #         acc_obstacle3.scale.z = 0.5
    #         acc_obstacle3.pose.orientation.x = 0.0
    #         acc_obstacle3.pose.orientation.y = 0.0
    #         acc_obstacle3.pose.orientation.z = 0.0
    #         acc_obstacle3.pose.orientation.w = 1.0
    #         acc_obstacle3.color.r = 1.0
    #         acc_obstacle3.color.g = 1.0
    #         acc_obstacle3.color.b = 0.0
    #         acc_obstacle3.color.a = 0.5
    #         acc_obstacle3.pose.position.x = self.collidable_obstacle_info[i][6]
    #         acc_obstacle3.pose.position.y = self.collidable_obstacle_info[i][8]
    #         acc_obstacle3.pose.position.z = 0.0


    #         acc_obstacle.lifetime = rospy.Duration(0.1)
    #         acc_obstacle2.lifetime = rospy.Duration(0.1)
    #         acc_obstacle3.lifetime = rospy.Duration(0.1)
    #         acc_obstacle_array.markers.append(acc_obstacle)
    #         acc_obstacle_array.markers.append(acc_obstacle2)
    #         acc_obstacle_array.markers.append(acc_obstacle3)

    #     self.acc_obstacle_pub.publish(acc_obstacle_array)

    def visualizeAccObstacle(self):
        acc_obstacle_array = MarkerArray()
        # print(self.center_distance_threshold)
        for i in range(len(self.collidable_obstacle_info)):
            center = Marker()
            center.header.frame_id = "map"
            center.id = i * 5  # 고유 ID 설정
            center.type = center.CYLINDER
            center.action = center.ADD
            center.scale.x = self.collidable_obstacle_info[i][-1] * 2
            center.scale.y = self.collidable_obstacle_info[i][-1] * 2
            center.scale.z = 0.5
            center.pose.orientation.x = 0.0
            center.pose.orientation.y = 0.0
            center.pose.orientation.z = 0.0
            center.pose.orientation.w = 1.0
            center.color.r = 1.0
            center.color.g = 1.0
            center.color.b = 1.0
            center.color.a = 0.5
            center.pose.position.x = self.collidable_obstacle_info[i][0]
            center.pose.position.y = self.collidable_obstacle_info[i][1]
            center.pose.position.z = self.collidable_obstacle_info[i][2]


            left_top = Marker()
            left_top.header.frame_id = "map"
            left_top.id = i * 5 + 1  # 고유 ID 설정
            left_top.type = left_top.CYLINDER
            left_top.action = left_top.ADD
            left_top.scale.x = self.angle_distance_threshold * 2
            left_top.scale.y = self.angle_distance_threshold * 2
            left_top.scale.z = 0.5
            left_top.pose.orientation.x = 0.0
            left_top.pose.orientation.y = 0.0
            left_top.pose.orientation.z = 0.0
            left_top.pose.orientation.w = 1.0
            left_top.color.r = 0.0
            left_top.color.g = 1.0
            left_top.color.b = 0.0
            left_top.color.a = 0.5
            left_top.pose.position.x = self.collidable_obstacle_info[i][6]
            left_top.pose.position.y = self.collidable_obstacle_info[i][7]
            left_top.pose.position.z = 0.0

            left_bottom = Marker()
            left_bottom.header.frame_id = "map"
            left_bottom.id = i * 5 + 2  # 고유 ID 설정
            left_bottom.type = left_top.CYLINDER
            left_bottom.action = left_top.ADD
            left_bottom.scale.x = self.angle_distance_threshold * 2
            left_bottom.scale.y = self.angle_distance_threshold * 2
            left_bottom.scale.z = 0.5
            left_bottom.pose.orientation.x = 0.0
            left_bottom.pose.orientation.y = 0.0
            left_bottom.pose.orientation.z = 0.0
            left_bottom.pose.orientation.w = 1.0
            left_bottom.color.r = 0.0
            left_bottom.color.g = 1.0
            left_bottom.color.b = 0.0
            left_bottom.color.a = 0.5
            left_bottom.pose.position.x = self.collidable_obstacle_info[i][8]
            left_bottom.pose.position.y = self.collidable_obstacle_info[i][9]
            left_bottom.pose.position.z = 0.0

            right_top = Marker()
            right_top.header.frame_id = "map"
            right_top.id = i * 5 + 3  # 고유 ID 설정
            right_top.type = right_top.CYLINDER
            right_top.action = right_top.ADD
            right_top.scale.x = self.angle_distance_threshold * 2
            right_top.scale.y = self.angle_distance_threshold * 2
            right_top.scale.z = 0.5
            right_top.pose.orientation.x = 0.0
            right_top.pose.orientation.y = 0.0
            right_top.pose.orientation.z = 0.0
            right_top.pose.orientation.w = 1.0
            right_top.color.r = 0.0
            right_top.color.g = 1.0
            right_top.color.b = 0.0
            right_top.color.a = 0.5
            right_top.pose.position.x = self.collidable_obstacle_info[i][10]
            right_top.pose.position.y = self.collidable_obstacle_info[i][11]
            right_top.pose.position.z = 0.0

            right_bottom = Marker()
            right_bottom.header.frame_id = "map"
            right_bottom.id = i * 5 + 4  # 고유 ID 설정
            right_bottom.type = right_top.CYLINDER
            right_bottom.action = right_top.ADD
            right_bottom.scale.x = self.angle_distance_threshold * 2
            right_bottom.scale.y = self.angle_distance_threshold * 2
            right_bottom.scale.z = 0.5
            right_bottom.pose.orientation.x = 0.0
            right_bottom.pose.orientation.y = 0.0
            right_bottom.pose.orientation.z = 0.0
            right_bottom.pose.orientation.w = 1.0
            right_bottom.color.r = 0.0
            right_bottom.color.g = 1.0
            right_bottom.color.b = 0.0
            right_bottom.color.a = 0.5
            right_bottom.pose.position.x = self.collidable_obstacle_info[i][12]
            right_bottom.pose.position.y = self.collidable_obstacle_info[i][13]
            right_bottom.pose.position.z = 0.0


            center.lifetime = rospy.Duration(0.1)
            left_top.lifetime = rospy.Duration(0.1)
            left_bottom.lifetime = rospy.Duration(0.1)
            right_top.lifetime = rospy.Duration(0.1)
            right_bottom.lifetime = rospy.Duration(0.1)

            acc_obstacle_array.markers.append(center)
            acc_obstacle_array.markers.append(left_top)
            acc_obstacle_array.markers.append(left_bottom)
            acc_obstacle_array.markers.append(right_top)
            acc_obstacle_array.markers.append(right_bottom)


        self.acc_obstacle_pub.publish(acc_obstacle_array)

    def visualizeTargetPoint(self):
        pure_pursuit_target_point = Marker()
        pure_pursuit_target_point.header.frame_id = "map"
        pure_pursuit_target_point.type = pure_pursuit_target_point.SPHERE
        pure_pursuit_target_point.action = pure_pursuit_target_point.ADD
        pure_pursuit_target_point.scale.x = 1.0
        pure_pursuit_target_point.scale.y = 1.0
        pure_pursuit_target_point.scale.z = 1.0
        pure_pursuit_target_point.pose.orientation.w = 1.0
        pure_pursuit_target_point.color.r = 1.0
        pure_pursuit_target_point.color.g = 0.0
        pure_pursuit_target_point.color.b = 0.0
        pure_pursuit_target_point.color.a = 1.0 
        pure_pursuit_target_point.pose.position.x = self.target_x
        pure_pursuit_target_point.pose.position.y = self.target_y
        pure_pursuit_target_point.pose.position.z = 0.0
        
        self.pure_pursuit_target_point_pub.publish(pure_pursuit_target_point)

    def visualizeCurvatureTargetPoint(self):
        curvature_target_point = Marker()
        curvature_target_point.header.frame_id = "map"
        curvature_target_point.type = curvature_target_point.SPHERE
        curvature_target_point.action = curvature_target_point.ADD
        curvature_target_point.scale.x = 1.0
        curvature_target_point.scale.y = 1.0
        curvature_target_point.scale.z = 1.0
        curvature_target_point.pose.orientation.w = 1.0
        curvature_target_point.color.r = 0.0
        curvature_target_point.color.g = 0.0
        curvature_target_point.color.b = 1.0
        curvature_target_point.color.a = 1.0 
        curvature_target_point.pose.position.x = self.curvature_target_x
        curvature_target_point.pose.position.y = self.curvature_target_y
        curvature_target_point.pose.position.z = 0.0
        
        self.curvature_target_point_pub.publish(curvature_target_point)

    def visualizeEgoMarker(self):
        ego_marker = Marker()
        ego_marker.header.frame_id = "map"
        ego_marker.type = ego_marker.MESH_RESOURCE
        ego_marker.mesh_resource = "package://pure_pursuit/stl/egolf.stl"
        ego_marker.mesh_use_embedded_materials = True
        ego_marker.action = ego_marker.ADD
        ego_marker.scale.x = 1.2
        ego_marker.scale.y = 1.2
        ego_marker.scale.z = 1.2
        ego_marker.pose.orientation.x = self.quaternion_data[0]
        ego_marker.pose.orientation.y = self.quaternion_data[1] 
        ego_marker.pose.orientation.z = self.quaternion_data[2]
        ego_marker.pose.orientation.w = self.quaternion_data[3]
        ego_marker.color.r = 1.0
        ego_marker.color.g = 1.0
        ego_marker.color.b = 1.0
        ego_marker.color.a = 1.0
        ego_marker.pose.position.x = self.status_msg.position.x
        ego_marker.pose.position.y = self.status_msg.position.y
        ego_marker.pose.position.z = 0.0
        
        self.ego_marker_pub.publish(ego_marker)


    def isRedLight(self):
        # print(f"red {self.red_light_count} green {self.green_light_count}")
        # if self.green_light_count >= 25 and self.red_light_count <= 5:
        #     return False  # Green
        # else:  
        #     return True   # Red

        if self.green_light_count < 25 and self.red_light_count > 25:
            return True  # Red
        else:  
            return False # Green


    def isTrafficLightArea(self):
        if ((self.traffic_stop_index_1 - self.max_velocity/3 <= self.current_waypoint <= self.traffic_stop_index_1) or (self.traffic_stop_index_2 - self.max_velocity/3 <= self.current_waypoint <= self.traffic_stop_index_2) ) :#(self.current_waypoint <= self.traffic_stop_index_1+10) or 
            return True
        else:
            return False
    

if __name__ == '__main__':
    try:
        pure_pursuit_= PurePursuit()
    except rospy.ROSInterruptException:
        pass