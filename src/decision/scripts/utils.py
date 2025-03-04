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
 
    def isObstacleOnPath(self, ref_path, global_valid_obstacle, vehicle_status, current_waypoint,is_path_switching): 
        return
    
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