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

back_waypoint = 10

class pathReader :  ## 텍스트 파일에서 경로를 출력 ##
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/sangam/"+file_name
        openFile = open(full_file_name, 'r')
        out_path_control=Path()

        out_path_control.header.frame_id='map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.position.z=0
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path_control.poses.append(read_pose)

        openFile.close()
        return out_path_control ## 읽어온 경로를 global_path로 반환 ##
    
def findCurrentwaypoint(ref_path, status_msg):
    current_x=status_msg.position.x
    current_y=status_msg.position.y
    current_waypoint=0
    min_dis=float('inf')

    # waypoint_counts = 100 # 기존 주행 코스 50 최적값.
    # waypoint_counts = 80

    for i in range(len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i
    
    return current_waypoint



def findLocalPath(ref_path,status_msg): ## global_path와 차량의 status_msg를 이용해 현재 waypoint와 local_path를 생성 ##
    out_path_control=Path()
    current_x=status_msg.position.x
    current_y=status_msg.position.y
    current_waypoint=0
    min_dis=float('inf')

    # waypoint_counts = 100 # 기존 주행 코스 50 최적값.
    waypoint_counts = 80

    for i in range(len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i



    if current_waypoint + waypoint_counts > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)
    else :
        last_local_waypoint=current_waypoint + waypoint_counts


    
    out_path_control.header.frame_id='map'
    # fifteen_past_path.header.frame_id = 'map'
    for i in range(current_waypoint - back_waypoint, last_local_waypoint) :
        if i < 0:
            continue

        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=1
        out_path_control.poses.append(tmp_pose)

    return out_path_control, min_dis # current_waypoint  ## local_path와 waypoint를 반환 ##



class purePursuit : ## purePursuit 알고리즘 적용 ##
    def __init__(self):
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=4.635 # 4.6 # 3.0
        self.lfd = 7
        self.min_lfd = 5.0
        self.max_lfd = 12.0
        self.steering = 0
        
        self.is_obstacle_passed = False
        self.first_clock_wise = None

        # self.previous_lattice_weights = [0, 0, 0]
        # self.previous_selected_lane = 0

    def getPath(self,msg):
        self.path=msg  #nav_msgs/Path 
    
    
    def getEgoStatus(self, msg):

        self.current_vel=msg.velocity.x  #kph
        self.vehicle_yaw=(msg.heading)/180*pi   # rad
        self.current_postion.x=msg.position.x ## 차량의 현재x 좌표 ##
        self.current_postion.y=msg.position.y ## 차량의 현재y 좌표 ##
        self.current_postion.z=0.0 ## 차량의 현재z 좌표 ##


    def steeringAngle(self, is_path_switching, static_lfd=0, mission_name = " ", min_distance = 0):  ## purePursuit 알고리즘을 이용한 Steering 계산 ## 
        vehicle_position=self.current_postion
        rotated_point=Point()
        self.is_look_forward_point= False
        # print("self.lfd ",self.lfd)

        rotated_x_threshold = 0.0
        
        for i in self.path.poses: # self.path == local_path 
            path_point=i.pose.position
            dx = path_point.x - vehicle_position.x
            dy = path_point.y - vehicle_position.y
            rotated_point.x=cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
            rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy

            if rotated_point.x > rotated_x_threshold :
                dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))

                if static_lfd > 0:
                    self.lfd = static_lfd
                
                if dis >= self.lfd :
                    # print("VEL:", self.current_vel)
                    self.lfd=self.current_vel * 0.2 # sangam
                    if self.lfd < self.min_lfd : 
                        self.lfd=self.min_lfd 

                    elif self.lfd > self.max_lfd :
                        self.lfd=self.max_lfd
                    if is_path_switching:
                        self.lfd += min_distance * 1.1
                    # print("LFD:", self.lfd)

                    self.forward_point=path_point
                    self.is_look_forward_point=True
                    
                    break
        
        theta=atan2(rotated_point.y,rotated_point.x)
    
        if (min_distance >= 3.0) and mission_name == "acc":
            # print("fixed_lfd")
            self.lfd = 9.5     # 8
        
        
        
        if self.is_look_forward_point :
            self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi * -1 #deg
            # print("lfd",self.lfd)
            return self.steering, self.forward_point.x, self.forward_point.y, self.lfd ## Steering 반환 ##
        else : 
            return 0, 0, 0, 0
        
        

    def estimateCurvature(self):
        vehicle_position = self.current_postion
        try:
            last_path_point = self.path.poses[-24].pose.position
        except:
            last_path_point = self.path.poses[-1].pose.position

        dx = last_path_point.x - vehicle_position.x
        dy = last_path_point.y - vehicle_position.y

        rotated_point=Point()
        rotated_point.x=cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
        rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
    
        self.far_foward_point = last_path_point

        corner_theta = abs(atan2(rotated_point.y,rotated_point.x))
        corner_theta_degree = corner_theta * 180 /pi

        return corner_theta_degree, self.far_foward_point.x, self.far_foward_point.y


    # def getMinDistance(self, ref_path, obstacle_info, vehicle_status):
        
    #     min_distance = 99999
    #     min_path_coord= [0, 0]
    #     min_obstacle_coord = [0, 0]

    #     for obstacle in obstacle_info:

    #         for path_pos in ref_path.poses:

    #             distance_from_path= sqrt(pow(obstacle[0]-path_pos.pose.position.x,2)+pow(obstacle[1]-path_pos.pose.position.y,2))
    #             distance_from_vehicle = max(sqrt((obstacle[0]-vehicle_status.position.x)**2 + (obstacle[1]-vehicle_status.position.y)**2),0.1)
    #             if distance_from_path < min_distance:
    #                 min_distance = distance_from_path
    #                 min_path_coord = [path_pos.pose.position.x, path_pos.pose.position.y]
    #                 min_obstacle_coord = [obstacle[0], obstacle[1]]
                        

    #     return min_distance, min_path_coord, min_obstacle_coord
    

    # def checkDynamicObstacle(self, clock_wise, min_distance, current_waypoint):
    #     is_dynamic_obstacle = False    
    #     distance_threshold = 8.0 # 4.5

    #     # 오른쪽에서 오는애면 줄이기
    #     if self.first_clock_wise == 1:
    #         if 3380 <= current_waypoint <= 3700:
    #             distance_threshold = 3.5 # 3.0
    #         else:
    #             distance_threshold = 2.5

        
    #     if self.first_clock_wise != None:
    #         if self.is_obstacle_passed == False:
    #             if (self.first_clock_wise * clock_wise) < 0:
    #                 distance_threshold = 0.75 #
    #                 self.is_obstacle_passed = True
    #             else:
    #                 self.is_obstacle_passed = False
                
    #         elif self.is_obstacle_passed == True:
    #             distance_threshold = 0.75
    #     else:
    #         self.first_clock_wise = clock_wise

    #     if min_distance <= distance_threshold:
    #         is_dynamic_obstacle = True
    #     else:
    #         is_dynamic_obstacle = False

    #     return is_dynamic_obstacle, distance_threshold
    

    # def isObstacleOnPath(self, ref_path, global_valid_obstacle, vehicle_status, current_waypoint): 
    #     is_obstacle_on_path = False
    #     distance_object_to_car_list = []
    #     collidable_obstacle_info = []
     
    #     distance_threshold = 2.7
    #     distance_threshold_corner = 1.6
    #     min_distance_threshold = 1.4

    #     vehicle_width = 0.7   # 2.0   1.8이 best 값  1.6으로 해도 지금 상당히 잘됨 !!

    #     # if (1448 <= current_waypoint <= 1555):   # 자동차 처리
    #     #     distance_threshold = 1.0

    #     # print(global_valid_obstacle)
    #     if len(global_valid_obstacle) > 0:
            
    #         for obstacle in global_valid_obstacle:
    #             min_dis = 99999
    #             distance_threshold = 2.7
    #             min_distance_threshold = 1.4

    #             for path_pos in ref_path.poses:
         
    #                 y_radius = obstacle[4]/2
                    
    #                 # 세 개의 dis 값 계산
    #                 # obstacle = center_x center_y center_z length_x length_y length_z min_x left_y right_y

    #                 dis1= sqrt(pow(obstacle[0]-path_pos.pose.position.x,2)+pow(obstacle[1]-path_pos.pose.position.y,2))
    #                 dis2= sqrt(pow(obstacle[6]-path_pos.pose.position.x,2)+pow(obstacle[7]-path_pos.pose.position.y,2))
    #                 dis3= sqrt(pow(obstacle[6]-path_pos.pose.position.x,2)+pow(obstacle[8]-path_pos.pose.position.y,2))
                    
    #                 min_dis = min(dis1, dis2, dis3, min_dis)

    #                 distance_object_to_car = max(sqrt((obstacle[0]-vehicle_status.position.x)**2 + (obstacle[1]-vehicle_status.position.y)**2), 0.1)
    #                 distance_object_to_car_list.append(distance_object_to_car)

    #                 if obstacle[4] >= 5.0:
    #                     if current_waypoint >= 2370:
    #                         distance_threshold = 2.0
    #                     else:
    #                         distance_threshold = 4.0
    #                     # collidable_obstacle_info.append(obstacle)
                    
    #                 elif obstacle[4] <= 0.5:
    #                     if (2000 <= current_waypoint <= 2030):   # 자동차 처리
    #                         distance_threshold = 1.0
    #                     else:
    #                         distance_threshold = 4.0
                    
    #                 else:
    #                     if current_waypoint >= 2370:
    #                         distance_threshold = 2.0
    #                     else:
    #                         distance_threshold = y_radius + vehicle_width


    #                 if distance_threshold < min_distance_threshold:
    #                     distance_threshold = min_distance_threshold

                    
    #                 # if dis2 <= distance_threshold_corner or dis3 <=distance_threshold_corner :
    #                 #     print("옆부분이 침 ")
    #                 #     is_obstacle_on_path = True
    #                 #     collidable_obstacle_info.append(obstacle)
    #                 #     break

    #                 # elif dis1 <= distance_threshold:
    #                 #     if all_blocked:  
    #                 #         print("중앙이 쳤지만 collsion bool all true여서 그냥 corner 만 체크해서 감")
    #                 #         break
    #                 #     else: 
    #                 #         print("중앙이 침 ", distance_threshold)
    #                 #         is_obstacle_on_path = True
    #                 #         collidable_obstacle_info.append(obstacle)
    #                 #         break

    #             if min_dis <= distance_threshold :
    #                 is_obstacle_on_path = True
    #                 collidable_obstacle_info.append(obstacle)
                     
        
    #     return is_obstacle_on_path, distance_object_to_car_list, distance_threshold, collidable_obstacle_info

    def isObstacleOnPath(self, ref_path, global_valid_obstacle, vehicle_status, current_waypoint,is_path_switching): 
        is_obstacle_on_path = False
        distance_object_to_car_list = []
        collidable_obstacle_info = []
     
        center_distance_threshold = 2.7
        angle_distance_threshold = 1.9

        center_min_distance_threshold = 1.4
        angle_min_distance_threshold = 1.0

        vehicle_width = 1.5

        if len(global_valid_obstacle) > 0:
            
            for obstacle in global_valid_obstacle:
                min_dis_center = 99999
                min_dis_angle  = 99999

                center_distance_threshold = 2.7
                angle_distance_threshold = 1.9

                center_min_distance_threshold = 1.4

                center_x, center_y, center_z   = obstacle[0],  obstacle[1],  obstacle[2]
                length_x, length_y, length_z   = obstacle[3],  obstacle[4],  obstacle[5]

                left_top_x,     left_top_y     = obstacle[6],  obstacle[7]
                left_bottom_x,  left_bottom_y  = obstacle[8],  obstacle[9]
                right_top_x,    right_top_y    = obstacle[10], obstacle[11]
                right_bottom_x, right_bottom_y = obstacle[12], obstacle[13]  

                for path_pos in ref_path.poses:
         
                    y_radius = length_y/2
    
                    dis_center       = sqrt(pow(center_x       - path_pos.pose.position.x, 2) + pow(center_y       - path_pos.pose.position.y, 2))
                    dis_left_top     = sqrt(pow(left_top_x     - path_pos.pose.position.x, 2) + pow(left_top_y     - path_pos.pose.position.y, 2))
                    dis_left_bottom  = sqrt(pow(left_bottom_x  - path_pos.pose.position.x, 2) + pow(left_bottom_y  - path_pos.pose.position.y, 2))
                    dis_right_top    = sqrt(pow(right_top_x    - path_pos.pose.position.x, 2) + pow(right_top_y    - path_pos.pose.position.y, 2))
                    dis_right_bottom = sqrt(pow(right_bottom_x - path_pos.pose.position.x, 2) + pow(right_bottom_y - path_pos.pose.position.y, 2))
                
                    min_dis_center = min(dis_center, min_dis_center)
                    min_dis_angle = min(dis_left_top, dis_left_bottom, dis_right_top, dis_right_bottom, min_dis_angle)

                    distance_object_to_car = max(sqrt((center_x - vehicle_status.position.x)**2 + (center_y - vehicle_status.position.y)**2), 0.1)

                    if length_y >= 5.0:
                        if current_waypoint >= 2370:           # 종료 직전
                            center_distance_threshold = 2.0
                        else:
                            center_distance_threshold = 4.0    # 버스 처리

                    elif length_y <= 0.5:
                        if (2000 <= current_waypoint <= 2030): # 자동차 처리
                            center_distance_threshold = 1.0
                        else:
                            if (current_waypoint <= 50) :
                                center_distance_threshold = 2.0  # 코너 처리
                            # else:
                            #     center_distance_threshold = 4.5  # 보행자 처리
                    
                            
                    
                    else:
                        if current_waypoint >= 2370:           # 종료 직전
                            center_distance_threshold = 2.0 
                        else:                                  
                            center_distance_threshold = y_radius + vehicle_width  # 일반 상황     
                        
                       

                       
                   
        
                    if center_distance_threshold < center_min_distance_threshold: # 하한선 예외처리
                        center_distance_threshold = center_min_distance_threshold

                    angle_distance_threshold = center_distance_threshold * 0.7
                    
                     

                    if angle_distance_threshold < angle_min_distance_threshold:
                        angle_distance_threshold = angle_min_distance_threshold

                    if is_path_switching:
                        angle_distance_threshold = 0.0
                        center_distance_threshold = 1.0
                    
                    if min_dis_center <= center_distance_threshold or min_dis_angle <= angle_distance_threshold: # 장애물이 threshold 보다 패스에 가까울 때 즉, 충돌 위험이 있을 때 (장애물의 중심점이나 꼭짓점 중 아무거나 걸려도 충돌 위험이 있다고 판단함)
                        # print(min_dis_center, center_distance_threshold)
                        is_obstacle_on_path = True
                      
                        distance_object_to_car_list.append(distance_object_to_car)

                        obstacle.append(distance_object_to_car)
                        obstacle.append(center_distance_threshold)
                        collidable_obstacle_info.append(obstacle)
                        break

            # print(center_distance_threshold)
        
        return is_obstacle_on_path, distance_object_to_car_list, center_distance_threshold, angle_distance_threshold, collidable_obstacle_info

    

    def isObstacleOnRemovePath(self, ref_path, global_valid_obstacle, current_waypoint):
        is_obstacle_on_removepath = False
        distance_object_to_remove_list = []
        distance_object_to_remove_list_final = []
        
        threshold = 1.5     #0.9 
        
        # if current_waypoint <= 50: 
        #     threshold = 1.2
        if 1000 <= current_waypoint <= 1070:     # 오른쪽 차량 진입구간 대비
            threshold = 0.0

        if 1475 <= current_waypoint <= 1550:     # grandeur
            threshold = 4.5
        
        if 1610 <= current_waypoint <= 1790: # 우측 펜스 처리
            threshold = 2.0

        if 1995 <= current_waypoint <= 2020:    # 2010 ~ 2035
            threshold = 2.0
        
        if 2020 <= current_waypoint <= 2035:
            threshold = 0.0
        # if 2010 <= current_waypoint <= 2035:
        #     threshold = 1.8
        # print(global_valid_obstacle)
        
        if len(global_valid_obstacle) > 0:

            for obstacle in global_valid_obstacle:
                min_dis = float('inf')
                for i in range(len(ref_path.poses)):
                    dx = obstacle[0] - ref_path.poses[i].pose.position.x
                    dy = obstacle[1] - ref_path.poses[i].pose.position.y
                    # print("dx dy",dx,dy)
                    if sqrt(dx*dx + dy*dy) <= min_dis:
                        min_dis = sqrt(dx*dx + dy*dy)
                    
                # print(min_dis)

                if min_dis >= threshold: # remove 패스에 가까이 있지 않은 장애물은 삭제하지 않고 살리기
  
                    distance_object_to_remove_list.append(obstacle)
                

            # if 800<= current_waypoint <= 1070: 
            #     for obstacle in distance_object_to_remove_list:
            #         min_dis = float('inf')
            #         for i in range(len(ref_path_2.poses)):
            #             dx = obstacle[0] - ref_path_2.poses[i].pose.position.x
            #             dy = obstacle[1] - ref_path_2.poses[i].pose.position.y
            #             if sqrt(dx*dx + dy*dy) <= min_dis:
            #                 min_dis = sqrt(dx*dx + dy*dy)
                    
            #         if min_dis >= threshold:
            #             distance_object_to_remove_list_final.append(obstacle)
            #             print("!!!!!!!!!!!!!")
            # else : 
            #     return distance_object_to_remove_list


        
                                                          
        return distance_object_to_remove_list





        #     for path_pos in ref_path.poses:
        #         for obstacle in global_valid_obstacle:
        #             dis= sqrt(pow(obstacle[0]-path_pos.pose.position.x,2)+pow(obstacle[1]-path_pos.pose.position.y,2))
        #             # if dis 
        #             # distance_object_to_car = max(sqrt((obstacle[0]-vehicle_status.position.x)**2 + (obstacle[1]-vehicle_status.position.y)**2), 0.1)

        #             # distance_object_to_remove_list.append(distance_object_to_car)

        #             # if dis <= distance_threshold :
        #             #     is_obstacle_on_path = True
                    
                        
        
        # return dis   
    


    def isObstacleOnLeftRemovePath(self, ref_path, global_valid_obstacle, current_waypoint):
        
        distance_object_to_remove_list = []
       
        
        threshold = 2.5
        
        # if 800 <=current_waypoint <= 1070 :# if current_waypoint <= 50: 
       
        
        if len(global_valid_obstacle) > 0:

            for obstacle in global_valid_obstacle:
                min_dis = float('inf')
                for i in range(len(ref_path.poses)):
                    dx = obstacle[0] - ref_path.poses[i].pose.position.x
                    dy = obstacle[1] - ref_path.poses[i].pose.position.y
                    # print("dx dy",dx,dy)
                    if sqrt(dx*dx + dy*dy) <= min_dis:
                        min_dis = sqrt(dx*dx + dy*dy)
                    
                # print(min_dis)

                if min_dis >= threshold: # remove 패스에 가까이 있지 않은 장애물은 삭제하지 않고 살리기

                    distance_object_to_remove_list.append(obstacle)
                
                
        return distance_object_to_remove_list
    
                                                          
        # return global_valid_obstacle
    ########################  lattice  ########################

    def latticePlanner(self, ref_path, global_vaild_object, vehicle_status, current_lane, current_waypoint):
        
        obstacle_info_lattice = []

        distance_threshold = 2.7     # 2.7

        if current_waypoint >= 1280:
            min_distance_threshold = 1.6  # 1.5 1.8
        else:
             min_distance_threshold = 2.0    # 2.7이었지만 첫번째 구간에서 PE-drum때문에 줄여놓음


       
        # max_distance_threshold = 6.0

        out_path_control=[]
        out_path_planning=[]

        selected_lane = -1
        lattice_current_lane = current_lane
        look_distance = int(vehicle_status.velocity.x * 0.43 + 8) #* 0.7)


        # look_distance = 30
        
        if len(ref_path.poses)>look_distance :
            # control path
            end_of_local_path_idx = back_waypoint+look_distance
            if end_of_local_path_idx >= len(ref_path.poses):
                end_of_local_path_idx = -1

            global_ref_start_point=(ref_path.poses[back_waypoint].pose.position.x,ref_path.poses[back_waypoint].pose.position.y)
            global_ref_start_next_point=(ref_path.poses[back_waypoint+1].pose.position.x,ref_path.poses[back_waypoint+1].pose.position.y)
            global_ref_end_point=(ref_path.poses[end_of_local_path_idx].pose.position.x,ref_path.poses[end_of_local_path_idx].pose.position.y)
            
            theta=atan2(global_ref_start_next_point[1]-global_ref_start_point[1],global_ref_start_next_point[0]-global_ref_start_point[0])
            translation=[global_ref_start_point[0],global_ref_start_point[1]]

            t=np.array([[cos(theta), -sin(theta),translation[0]],[sin(theta),cos(theta),translation[1]],[0,0,1]])
            det_t=np.array([[t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])   ],[t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])   ],[0,0,1]])

            world_end_point=np.array([[global_ref_end_point[0]],[global_ref_end_point[1]],[1]])
            local_end_point=det_t.dot(world_end_point)

            world_start_point=np.array([[global_ref_start_point[0]],[global_ref_start_point[1]],[1]])
            # local_start_point=det_t.dot(world_start_point)

            # planning path
            global_ref_start_point_2=(ref_path.poses[0].pose.position.x,ref_path.poses[0].pose.position.y)
            global_ref_start_next_point_2=(ref_path.poses[1].pose.position.x,ref_path.poses[1].pose.position.y)
            global_ref_end_point_2=(ref_path.poses[0].pose.position.x,ref_path.poses[0].pose.position.y)
            
            theta_2=atan2(global_ref_start_next_point_2[1]-global_ref_start_point_2[1],global_ref_start_next_point_2[0]-global_ref_start_point_2[0])
            translation_2=[global_ref_start_point_2[0],global_ref_start_point_2[1]]

            t_2=np.array([[cos(theta_2), -sin(theta_2),translation_2[0]],[sin(theta_2),cos(theta_2),translation_2[1]],[0,0,1]])
            det_t_2=np.array([[t_2[0][0],t_2[1][0],-(t_2[0][0]*translation_2[0]+t_2[1][0]*translation_2[1])   ],[t_2[0][1],t_2[1][1],-(t_2[0][1]*translation_2[0]+t_2[1][1]*translation_2[1])   ],[0,0,1]])

            world_end_point_2=np.array([[global_ref_end_point_2[0]],[global_ref_end_point_2[1]],[1]])
            local_end_point_2=det_t_2.dot(world_end_point_2)

            world_start_point_2=np.array([[global_ref_start_point_2[0]],[global_ref_start_point_2[1]],[1]])
            # local_start_point_2=det_t_2.dot(world_start_point_2)

            # common
            world_ego_vehicle_position=np.array([[vehicle_status.position.x],[vehicle_status.position.y],[1]])
            local_ego_vehicle_position=det_t.dot(world_ego_vehicle_position)

            # lattice 간격
           
            if 1212 <= current_waypoint <= 1426:
                lane_off_set=[6.4, 3.2, 0]
                lane_weight = [2, 1, 0]     
            elif 1427 <= current_waypoint <= 1547:
                lane_off_set=[6.4, 3.2, 0, -3.2]
                lane_weight = [2, 1, 0, 1] 
            elif 1548 <= current_waypoint <= 1648:
                lane_off_set=[3.2, 0, -3.2]
                lane_weight = [1, 0, 1] 
            elif 1649 <= current_waypoint <= 1799:
                lane_off_set=[6.4, 3.2, 0]
                lane_weight = [2, 1, 0]
            # elif 1800 <= current_waypoint <= 2024:
            #     lane_off_set=[3.5, 0]
            #     lane_weight = [1, 0] 

            
            
            
            
            
            elif 1800 <= current_waypoint <=2019:
                lane_off_set=[3.5, 0]
                lane_weight = [1,0]


            elif 2020 <= current_waypoint <=2057:
                lane_off_set=[6.0, 3.5, 0, 0]
                lane_weight = [0, 2, 3, 1]

            # elif 2043 <= current_waypoint <=2052:
            #     lane_off_set=[3.5, 0, 0]
            #     lane_weight = [2, 0, 1]
            # elif 2056<= current_waypoint <=2072:
            #     lane_off_set=[3.2, 0, -0.5]
            #     lane_weight = [2, 0, 1]
            
            elif 2058 <= current_waypoint <=2063:
                lane_off_set=[3.5, 0, -0.9]
                lane_weight = [2, 1, 0]
            elif 2064 <= current_waypoint <=2068:
                lane_off_set=[3.5, 0, -0.7]
                lane_weight = [2, 0, 1]
            
            elif 2069 <= current_waypoint <=2072:
                lane_off_set=[3.5, 0, -0.5]
                lane_weight = [2, 0, 1]
             
           
           
           
           
           
           
           
           
           
            elif 2073 <= current_waypoint <= 2082:
                lane_off_set=[3.2, 0, -3.2]
                lane_weight = [2, 0, 1] 
            
            elif 2083 <= current_waypoint <= 2410:
                lane_off_set=[3.2, 0, -3.2]
                lane_weight = [2, 0, 1]    
            else:
                lane_off_set=[9.6, 6.4, 3.2, 0]
                lane_weight = [3, 2, 1, 0] 
                
            local_lattice_points_control=[]
            local_lattice_points_planning=[]

            ############################# lattice path 생성  #############################
            for i in range(len(lane_off_set)):
                # control path
                local_lattice_points_control.append([local_end_point[0][0], local_end_point[1][0]+lane_off_set[i], 1])

                # planning path
                local_lattice_points_planning.append([local_end_point_2[0][0], local_end_point_2[1][0]+lane_off_set[i], 1])

            for end_point in local_lattice_points_control :
                lattice_path=Path()
                lattice_path.header.frame_id='map'
                x=[]
                y=[]
                x_interval=0.01  # 0.3
                xs=0
                xf=end_point[0]
                ps=local_ego_vehicle_position[1][0]
 
                pf=end_point[1]
                x_num=xf/x_interval

                for i in range(xs,int(x_num)) : 
                    x.append(i*x_interval)
                
                a=[0.0,0.0,0.0,0.0]
                a[0]=ps
                a[1]=0
                a[2]=3.0*(pf-ps)/ ((xf*xf)+0.0000000001)
                a[3]=-2.0*(pf-ps)/ ((xf*xf*xf)+0.0000000001)

                for i in x:
                    result=a[3]*i*i*i+a[2]*i*i+a[1]*i+a[0]
                    y.append(result)


                for i in range(0,len(y)) :
                    local_result=np.array([[x[i]],[y[i]],[1]])
                    global_result=t.dot(local_result)

                    read_pose=PoseStamped()
                    read_pose.pose.position.x=global_result[0][0]
                    read_pose.pose.position.y=global_result[1][0]
                    read_pose.pose.position.z=0
                    read_pose.pose.orientation.x=0
                    read_pose.pose.orientation.y=0
                    read_pose.pose.orientation.z=0
                    read_pose.pose.orientation.w=1
                    lattice_path.poses.append(read_pose)

                out_path_control.append(lattice_path)

            for end_point in local_lattice_points_planning :
                lattice_path=Path()
                lattice_path.header.frame_id='map'
                x=[]
                y=[]
                x_interval=0.5  # 0.3
                xs=0
                xf=end_point[0]
                ps=local_ego_vehicle_position[1][0]
                # ps=local_start_point[1][0]
 
                pf=end_point[1]
                x_num=xf/x_interval

                for i in range(xs,int(x_num)) : 
                    x.append(i*x_interval)
                
                a=[0.0,0.0,0.0,0.0]
                a[0]=ps
                a[1]=0
                a[2]=3.0*(pf-ps)/ ((xf*xf)+0.0000000001)
                a[3]=-2.0*(pf-ps)/ ((xf*xf*xf)+0.0000000001)

                for i in x:
                    result=a[3]*i*i*i+a[2]*i*i+a[1]*i+a[0]
                    y.append(result)


                for i in range(0,len(y)) :
                    local_result=np.array([[x[i]],[y[i]],[1]])
                    global_result=t.dot(local_result)

                    read_pose=PoseStamped()
                    read_pose.pose.position.x=global_result[0][0]
                    read_pose.pose.position.y=global_result[1][0]
                    read_pose.pose.position.z=0
                    read_pose.pose.orientation.x=0
                    read_pose.pose.orientation.y=0
                    read_pose.pose.orientation.z=0
                    read_pose.pose.orientation.w=1
                    lattice_path.poses.append(read_pose)

                out_path_planning.append(lattice_path)
            ############################# lattice path 생성  #############################    

            # add_point_size = int(vehicle_status.velocity.x*2*3.6) + back_waypoint
            add_point_size = 250
            if add_point_size > len(ref_path.poses)-2:
                add_point_size = len(ref_path.poses)

            elif add_point_size < 10 :
                add_point_size = 10

            # add_point_size = 0

            # print('add point',add_point_size)
            
            # if 2006 <= current_waypoint <= 2024:
                

            for i in range(14+look_distance,add_point_size):
                if i+1 < len(ref_path.poses):  # 로컬 패스 종료지점까지만 그리기
                    tmp_theta=atan2(ref_path.poses[i+1].pose.position.y-ref_path.poses[i].pose.position.y,ref_path.poses[i+1].pose.position.x-ref_path.poses[i].pose.position.x)
                    # print(i, "tmp_theta", tmp_theta)
                    tmp_translation=[ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],[sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],[0,0,1]])
                    tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],[tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],[0,0,1]])

                    for lane_num in range(len(lane_off_set)) :
                    
                        local_result=np.array([[0],[lane_off_set[lane_num]],[1]])
                        # print("tmp_theta",tmp_theta)
                        if lane_num == 3 and (2020 <= current_waypoint <= 2057):
                            print("1")

                            tmp_theta = -0.55

                            # dy = ref_path.poses[1].pose.position.y - ref_path.poses[0].pose.position.y
                            # dx = ref_path.poses[1].pose.position.x - ref_path.poses[0].pose.position.x

                            dy = -0.2612152131061052 
                            dx = 0.42633435929889174
                            


                            tmp_path = []
                            for j in range(len(ref_path.poses)):
                                
                                if j > add_point_size:
                                    break
                                # if current_waypoint >= 2046:
                                #     tmp_path.append([526.6875531017805+ (dx*j), -47.349262020195674 + (dy*j)])
                                # else:
                                tmp_path.append([ref_path.poses[0].pose.position.x + (dx*j), ref_path.poses[0].pose.position.y + (dy*j)])

                            tmp_translation=[tmp_path[i][0],tmp_path[i][1]]
                            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],[sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],[0,0,1]])
                        
                        elif lane_num == 2 and (2058 <= current_waypoint <= 2072):
                            tmp_theta = -0.55
                            print("2")

                            # dy = ref_path.poses[1].pose.position.y - ref_path.poses[0].pose.position.y
                            # dx = ref_path.poses[1].pose.position.x - ref_path.poses[0].pose.position.x

                            dy = -0.2612152131061052 
                            dx = 0.42633435929889174
                            
                            # print(dy, dx,dy/dx)


                            tmp_path = []
                            for j in range(len(ref_path.poses)):
                                
                                if j > add_point_size:
                                    break
                                # if current_waypoint >= 2046:
                                #     tmp_path.append([526.6875531017805+ (dx*j), -47.349262020195674 + (dy*j)])
                                # else:
                                tmp_path.append([ref_path.poses[0].pose.position.x + (dx*j), ref_path.poses[0].pose.position.y + (dy*j)])

                            tmp_translation=[tmp_path[i][0],tmp_path[i][1]]
                            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],[sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],[0,0,1]])

                            
                        global_result=tmp_t.dot(local_result)

                        read_pose=PoseStamped()
                        read_pose.pose.position.x=global_result[0][0]
                        read_pose.pose.position.y=global_result[1][0]
                        read_pose.pose.position.z=0
                        read_pose.pose.orientation.x=0
                        read_pose.pose.orientation.y=0
                        read_pose.pose.orientation.z=0
                        read_pose.pose.orientation.w=1
                        out_path_control[lane_num].poses.append(read_pose)


            for i in range(0,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta=atan2(ref_path.poses[i+1].pose.position.y-ref_path.poses[i].pose.position.y,ref_path.poses[i+1].pose.position.x-ref_path.poses[i].pose.position.x)
                    
                    tmp_translation=[ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],[sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],[0,0,1]])
                    tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],[tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],[0,0,1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result=np.array([[0],[lane_off_set[lane_num]],[1]])

                        if lane_num == 3 and (2020 <= current_waypoint <= 2057):
                            tmp_theta = -0.55
                            
                            print("3")
                                
                            # print("ref",current_waypoint,ref_path.poses[0].pose.position.x , ref_path.poses[0].pose.position.y)

                            # dy = ref_path.poses[1].pose.position.y - ref_path.poses[0].pose.position.y
                            # dx = ref_path.poses[1].pose.position.x - ref_path.poses[0].pose.position.x

                            dy = -0.2612152131061052 
                            dx = 0.42633435929889174

                            tmp_path = []
                            for j in range(len(ref_path.poses)):
                                
                                if j > add_point_size:
                                    break
                                # if current_waypoint >= 2046:
                                #     tmp_path.append([526.6875531017805+ (dx*j), -47.349262020195674 + (dy*j)])
                                #     # print("!!!!!!!!!!!!!!!!1")
                                                                   
                                # else:
                                tmp_path.append([ref_path.poses[0].pose.position.x + (dx*j), ref_path.poses[0].pose.position.y + (dy*j)])
                            tmp_translation=[tmp_path[i][0],tmp_path[i][1]]
                            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],[sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],[0,0,1]])

                        
                        elif lane_num == 2 and (2058 <= current_waypoint <= 2072):
                            tmp_theta = -0.55
                            
                            print("4")
                            # print("ref",current_waypoint,ref_path.poses[0].pose.position.x , ref_path.poses[0].pose.position.y)

                            # dy = ref_path.poses[1].pose.position.y - ref_path.poses[0].pose.position.y
                            # dx = ref_path.poses[1].pose.position.x - ref_path.poses[0].pose.position.x

                            dy = -0.2612152131061052 
                            dx = 0.42633435929889174

                            tmp_path = []
                            for j in range(len(ref_path.poses)):
                                
                                if j > add_point_size:
                                    break
                                # if current_waypoint >= 2046:
                                #     tmp_path.append([526.6875531017805+ (dx*j), -47.349262020195674 + (dy*j)])
                                #     # print("!!!!!!!!!!!!!!!!1")
                                                                   
                                # else:
                                tmp_path.append([ref_path.poses[0].pose.position.x + (dx*j), ref_path.poses[0].pose.position.y + (dy*j)])
                               

                            tmp_translation=[tmp_path[i][0],tmp_path[i][1]]
                            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],[sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],[0,0,1]])

                        global_result=tmp_t.dot(local_result)

                        read_pose=PoseStamped()
                        read_pose.pose.position.x=global_result[0][0]
                        read_pose.pose.position.y=global_result[1][0]
                        read_pose.pose.position.z=0
                        read_pose.pose.orientation.x=0
                        read_pose.pose.orientation.y=0
                        read_pose.pose.orientation.z=0
                        read_pose.pose.orientation.w=1
                        out_path_planning[lane_num].poses.append(read_pose)


            # lane_weight=[2, 1, 0] #reference path 
            # lane_weight = [w for w in range(len(lane_off_set), 0, -1)]


            collision_bool=[False for _ in range(len(lane_off_set))]

            # lattice path 내 장애물 탐색하여 가중치 조절
            # print(len(global_vaild_object))
            if len(global_vaild_object)>0:
                # print(len(global_vaild_object))
                for obj in global_vaild_object :

                    y_radius = obj[4]/2  # obj[4] == object length y
                    # half_lane_width = 1.5     # 1.5
                    # quarter_lane_width = 0.8
                    vehicle_width = 1.5            #1.8  

                    for path_num in range(len(out_path_planning)) :
                        for path_pos in out_path_planning[path_num].poses : #path_pos = PoseStamped()
                            dis_obj_to_path = sqrt(pow(obj[0]-path_pos.pose.position.x, 2)+pow(obj[1]-path_pos.pose.position.y, 2))
                            dis_obj_to_car = max(sqrt((obj[0]-vehicle_status.position.x)**2 + (obj[1]-vehicle_status.position.y)**2),0.1)

                            # dis_obj1_to_path = sqrt(pow(obj[0]-path_pos.pose.position.x, 2)+pow(obj[1]-path_pos.pose.position.y, 2))
                            # dis_obj1_to_car = max(sqrt((obj[0]-vehicle_status.position.x)**2 + (obj[1]-vehicle_status.position.y)**2),0.1)
                            # dis_obj2_to_path = sqrt(pow(obj[6]-path_pos.pose.position.x, 2)+pow(obj[7]-path_pos.pose.position.y, 2))
                            # dis_obj2_to_car = max(sqrt((obj[6]-vehicle_status.position.x)**2 + (obj[7]-vehicle_status.position.y)**2),0.1)
                            # dis_obj3_to_path = sqrt(pow(obj[6]-path_pos.pose.position.x, 2)+pow(obj[8]-path_pos.pose.position.y, 2))
                            # dis_obj3_to_car = max(sqrt((obj[6]-vehicle_status.position.x)**2 + (obj[8]-vehicle_status.position.y)**2),0.1)
                            # dis_obj_to_path = min(dis_obj1_to_path, dis_obj2_to_path, dis_obj3_to_path)
                            # dis_obj_to_car = min(dis_obj1_to_car, dis_obj2_to_car, dis_obj3_to_car)

                            if (1448 <= current_waypoint <= 1555):
                                distance_threshold = 1.0
                                            
                            else:
                                if  1280 <= current_waypoint <= 1849:            #1780
                                    distance_threshold = y_radius + vehicle_width/2
                                
                                elif current_waypoint >= 1850 :
                                    distance_threshold = y_radius + vehicle_width/8
                                    if distance_threshold >= 1.7:
                                        distance_threshold = 1.7
                                    # print(f"path num : {path_num}, dist_threshold {distance_threshold}, min_distance_threshold {min_distance_threshold}")
                                else:
                                    distance_threshold = y_radius + vehicle_width 
                                    # print(distance_threshold)

                            # print(distance_threshold)
                            if distance_threshold < min_distance_threshold:
                                # print("lower than min")
                                distance_threshold = min_distance_threshold
                          
                            # print(dis_obj_to_path)
                            if dis_obj_to_path <= distance_threshold :
                                # print(dis_obj_to_path)
                                collision_bool[path_num] = True
                                lane_weight[path_num] += 2*dis_obj_to_path**-1 * (1000/dis_obj_to_car)

                                
                                # break

                    obj.append(distance_threshold)
                    obstacle_info_lattice.append(obj)

            selected_lane = lane_weight.index(min(lane_weight))

            all_lane_collision=True

            # print(distance_threshold)
        else :
            print("NO Reference Path")
            selected_lane = -1    
        # print("final",distance_threshold)
        return out_path_control, out_path_planning, selected_lane, distance_threshold, collision_bool, obstacle_info_lattice


    ########################  lattice  ########################


    def PathSwitchingPlanner(self, ref_path, ref_path1, ref_path2, global_valid_object, vehicle_status, selected_lane):
        min_distance_threshold = 1.6
    
        out_path_planning = [ref_path, ref_path1, ref_path2]
        lane_weight = [1, 2, 0]  # 각 경로에 대한 가중치 리스트
        front_blocked = False
        # obstacle_info_lattice = []  # 장애물 정보 저장 리스트
        
        # 장애물이 있을 경우 경로별 가중치 계산
        if len(global_valid_object) > 0:
            for obj in global_valid_object:
                y_radius = obj[4] / 2  # 장애물의 길이
                vehicle_width = 1.5  # 차량의 폭

                # print("계산전:", selected_lane)
                for path_pos in out_path_planning[selected_lane].poses:
                        dis_obj_to_path = sqrt(pow(obj[0]-path_pos.pose.position.x, 2)+pow(obj[1]-path_pos.pose.position.y, 2))
                        dis_obj_to_car = max(sqrt((obj[0]-vehicle_status.position.x)**2 + (obj[1]-vehicle_status.position.y)**2),0.1)
                
                        distance_threshold = y_radius + vehicle_width / 8
                        
                        if distance_threshold >= 1.7:
                            distance_threshold = 1.7
                        # 최소 거리 임계값 적용
                        if distance_threshold < min_distance_threshold:
                            distance_threshold = min_distance_threshold

                        # 장애물이 경로에 너무 가까운 경우 경로 가중치 증가
                        if dis_obj_to_path <= distance_threshold:
                            front_blocked = True 
                        else : # 내 앞에 장애물이 없을 떄
                            # print("현재 차선 그대로 유지")
                            # front_blocked = False
                            selected_lane = selected_lane
                if not front_blocked :
                    print("selected lane이 막혀있지 않음 ")
                if front_blocked: 
                
                    for path_num in range(len(out_path_planning)):
                        for path_pos in out_path_planning[path_num].poses:
                            # 경로 상의 위치와 장애물 간 거리 계산
                            dis_obj_to_path = sqrt(pow(obj[0]-path_pos.pose.position.x, 2)+pow(obj[1]-path_pos.pose.position.y, 2))
                            dis_obj_to_car = max(sqrt((obj[0]-vehicle_status.position.x)**2 + (obj[1]-vehicle_status.position.y)**2),0.1)
                            # print(dis_obj_to_path) 
                            # 현재 waypoint에 따른 거리 임계값 계산
                    
                            distance_threshold = y_radius + vehicle_width / 8
                            
                            if distance_threshold >= 1.7:
                                distance_threshold = 1.7
                            
                            # else:
                            #     distance_threshold = y_radius + vehicle_width

                            # 최소 거리 임계값 적용
                            if distance_threshold < min_distance_threshold:
                                distance_threshold = min_distance_threshold

                            # 장애물이 경로에 너무 가까운 경우 경로 가중치 증가
                            if dis_obj_to_path <= distance_threshold:
                                # print("!!!!!!!!!!!!!!")
                                lane_weight[path_num] += 2 * dis_obj_to_path ** -1 * (1000 / dis_obj_to_car)

                    # 장애물 정보 업데이트
                    # obj.append(distance_threshold)
                    # obstacle_info_lattice.append(obj)

        # 가장 가중치가 낮은 경로 선택
                    print("막혀있어서 weight 재계산", lane_weight)
                    selected_lane = lane_weight.index(min(lane_weight))
        print("최종 선택", selected_lane)
        print(lane_weight)
        return selected_lane




def CCW(vehicle_coord, min_path_coord, min_obstacle_coord):
    cross_product = (min_path_coord[0] - vehicle_coord.position.x) * (min_obstacle_coord[1] - min_path_coord[1]) - (min_path_coord[1] - vehicle_coord.position.y) * (min_obstacle_coord[0] - min_path_coord[0])

    if cross_product > 0:
        return -1 # 시계 반대방향인 경우
    elif cross_product < 0:
        return 1  # 시계방향인 경우
    else:
        return 0
    

def rotateLiDAR2GPS(obstacle_array, vehicle_status, current_waypoint=-1, current_lane=2) :

    lidar_x_position = 4.20 #1.44
    lidar_y_position = 0.0

    if len(obstacle_array) == 0: 
        return []

    lidar_in_enu_coord = []

    theta = vehicle_status.heading / 180*pi # vehicle heading

    for bbox in obstacle_array :
        # x = bbox[0]
        # y = bbox[1]
        # z = bbox[2]
        # length_x = bbox[3]
        # length_y = bbox[4]
        # length_z = bbox[5]

        # x_min =  bbox[6]
        # y_min =  bbox[7]
        # y_max =  bbox[8]

        # new_x = (x+lidar_x_position)*cos(theta) - (y+lidar_y_position)*sin(theta) + vehicle_status.position.x
        # new_y = (x+lidar_x_position)*sin(theta) + (y+lidar_y_position)*cos(theta) + vehicle_status.position.y


        # min_x = (x_min+lidar_x_position)*cos(theta) - (y_min+lidar_y_position)*sin(theta) + vehicle_status.position.x
        # left_y = (x_min+lidar_x_position)*sin(theta) + (y_min+lidar_y_position)*cos(theta) + vehicle_status.position.y
        # right_y = (x_min+lidar_x_position)*sin(theta) + (y_max+lidar_y_position)*cos(theta) + vehicle_status.position.y

        # lidar_in_enu_coord.append([new_x, new_y, z, length_x, length_y, length_z, min_x, left_y, right_y])

        center_x, center_y, center_z   = bbox[0],  bbox[1],  bbox[2]
        length_x, length_y, length_z   = bbox[3],  bbox[4],  bbox[5]

        left_top_x,     left_top_y     = bbox[6],  bbox[7]
        left_bottom_x,  left_bottom_y  = bbox[8],  bbox[9]
        right_top_x,    right_top_y    = bbox[10], bbox[11]
        right_bottom_x, right_bottom_y = bbox[12], bbox[13]  

        enu_center_x = (center_x+lidar_x_position)*cos(theta) - (center_y+lidar_y_position)*sin(theta) + vehicle_status.position.x
        enu_center_y = (center_x+lidar_x_position)*sin(theta) + (center_y+lidar_y_position)*cos(theta) + vehicle_status.position.y

        enu_left_top_x = (left_top_x+lidar_x_position)*cos(theta) - (left_top_y+lidar_y_position)*sin(theta) + vehicle_status.position.x
        enu_left_top_y = (left_top_x+lidar_x_position)*sin(theta) + (left_top_y+lidar_y_position)*cos(theta) + vehicle_status.position.y

        enu_left_bottom_x = (left_bottom_x+lidar_x_position)*cos(theta) - (left_bottom_y+lidar_y_position)*sin(theta) + vehicle_status.position.x
        enu_left_bottom_y = (left_bottom_x+lidar_x_position)*sin(theta) + (left_bottom_y+lidar_y_position)*cos(theta) + vehicle_status.position.y

        enu_right_top_x = (right_top_x+lidar_x_position)*cos(theta) - (right_top_y+lidar_y_position)*sin(theta) + vehicle_status.position.x
        enu_right_top_y = (right_top_x+lidar_x_position)*sin(theta) + (right_top_y+lidar_y_position)*cos(theta) + vehicle_status.position.y

        enu_right_bottom_x = (right_bottom_x+lidar_x_position)*cos(theta) - (right_bottom_y+lidar_y_position)*sin(theta) + vehicle_status.position.x
        enu_right_bottom_y = (right_bottom_x+lidar_x_position)*sin(theta) + (right_bottom_y+lidar_y_position)*cos(theta) + vehicle_status.position.y

        lidar_in_enu_coord.append([ enu_center_x, enu_center_y, center_z, 
                                    length_x, length_y, length_z, 
                                    enu_left_top_x, enu_left_top_y, # 6 7
                                    enu_left_bottom_x, enu_left_bottom_y, # 8 9 
                                    enu_right_top_x, enu_right_top_y, # 10 11
                                    enu_right_bottom_x, enu_right_bottom_y # 12 13
                                    ])


    return lidar_in_enu_coord
                    

class pidController : ## 속도 제어를 위한 PID 적용 ##
    def __init__(self):
        self.p_gain=0.1
        self.i_gain=0.0        
        self.d_gain=0.05
        self.controlTime=0.025 
        self.prev_error=0
        self.i_control=0


    def pid(self, target_velocity, current_velocity):
        error= target_velocity - current_velocity
        # print(error)
        
        p_control = self.p_gain * error
        self.i_control += self.i_gain *  error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error
        return output