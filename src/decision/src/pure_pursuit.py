#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import tf

from nav_msgs.msg import Path
from std_msgs.msg import Int64MultiArray, Float64, Int64, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import  Vector3
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import GPSMessage, CtrlCmd, EventInfo, EgoVehicleStatus
from morai_msgs.srv import MoraiEventCmdSrv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils import purePursuit, pidController

class EgoStatus:
    def __init__(self):
        self.position = Vector3()
        self.heading = 0.0
        self.velocity = Vector3()


class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # Publisher
        self.ego_marker_pub                 = rospy.Publisher('/ego_marker', Marker, queue_size=1)
        self.ctrl_cmd_pub                   = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.current_waypoint_pub           = rospy.Publisher('/current_waypoint', Int64, queue_size=1)
        self.pure_pursuit_target_point_pub  = rospy.Publisher('/pure_pusuit_target_point', Marker, queue_size=1)
        self.curvature_target_point_pub     = rospy.Publisher('/curvature_target_point', Marker, queue_size=1)

        ########################  lattice  ########################
        ########################  lattice  ########################

        # Subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.egoStatusCB) ## PID테스트 위해서 모라이에서 속도 받아오기  
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/local_path", Path, self.localpathCB)

        self.status_msg = EgoStatus()
        self.ctrl_cmd_msg = CtrlCmd()

        ### Param - Sensor Connection ###
        self.is_gps_status = False
        self.is_gps = False

        ### Param - Lateral Controller ###
        self.lfd = 0.0
        self.current_waypoint = [0.0, 0.0]
        self.target_x = 0.0
        self.target_y = 0.0
        self.curvature_target_x = 0.0
        self.curvature_target_y = 0.0
        self.corner_theta_degree = 0.0

        self.target_velocity = 5.0
        self.steering = 0.0
        ### Param - Longitudinal Controller ###
        self.accel_msg = 0.0
        self.brake_msg = 0.0
        self.pid_control_input = 0.0
        self.steering_offset = 0.015

        self.max_velocity = 10.0
    
        self.euler_data = [0,0,0,0]
        self.quaternion_data = [0,0,0,0]
        print("here?")
        ### Param - tf ### 
        self.proj_UTM = Proj(proj='utm', zone = 52, elips='WGS84', preserve_units=False)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # print("wait for service")
        # ######################################## For Service ########################################
        # rospy.wait_for_service('/Service_MoraiEventCmd')
        # self.req_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
        # self.req = EventInfo()
        # #############################################################################################
        # print("i think it's here")
        ### Class ###
        self.local_path = Path()
        # path_reader = pathReader('path_maker') ## 경로 파일의 위치
        self.pure_pursuit = purePursuit() ## purePursuit import
        pid = pidController()

        # ### Read path ###
        # self.global_path = path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름

        rate = rospy.Rate(40) 
        print("hello??")
        while not rospy.is_shutdown():
            print("hello?")
            self.getEgoCoord()
            if self.is_gps_status == True : 
                self.ctrl_cmd_msg.longlCmdType = 1
                print("im in while 1")
                self.pure_pursuit.getPath(self.local_path) #일단 local_path로만 주행 테스트 
                print("im in while 2")

                self.pure_pursuit.getEgoStatus(self.status_msg) # utils에서 계산하도록 속도, 위치 넘겨줌
                # @TODO getEgoStatus에 임시로 local path테스트만 하기에 0,0 으로 함수 내부에서 설정해둠 > 추후 수정
                
                self.steering, self.target_x, self.target_y, self.lfd = self.pure_pursuit.steeringAngle()
                self.corner_theta_degree, self.curvature_target_x, self.curvature_target_y = self.pure_pursuit.estimateCurvature()
                self.target_velocity = self.cornerController(self.corner_theta_degree)
                self.pid_control_input = pid.pid(self.target_velocity, self.current_velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)
                
                if self.pid_control_input > 0 :
                    self.accel_msg= self.pid_control_input
                    self.brake_msg = 0
                else :
                    self.accel_msg = 0
                    self.brake_msg = -self.pid_control_input

                self.steering_msg = (self.steering + 2.7) * self.steering_offset

                self.current_waypoint_pub.publish(self.current_waypoint)

                self.visualizeTargetPoint()
                self.publishCtrlCmd(self.accel_msg, self.steering_msg, self.brake_msg)

        rate.sleep()

    def getEgoCoord(self): ## Vehicle Status Subscriber 
        if self.is_gps == True:
            print("im getEgoCoord")
            self.status_msg.position.x = self.xy_zone[0] - 313008.55819800857
            self.status_msg.position.y = self.xy_zone[1] - 4161698.628368007
            self.status_msg.position.z = 0.0
            self.status_msg.heading = self.euler_data[2] * 180/np.pi
            self.status_msg.velocity.x = self.current_velocity

            self.tf_broadcaster.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                            tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*np.pi),
                            rospy.Time.now(),
                            "base_link",
                            "map")

            self.is_gps_status=True

        elif self.is_gps is False:
            self.status_msg.heading = self.euler_data[2] * 180/np.pi
            self.is_gps_status=False

        else:
            rospy.loginfo("Waiting for GPS")
            self.is_gps_status=False

    def localpathCB(self, msg): 
        print("i'm localpathCB")
        self.local_path = msg

    def gpsCB(self, msg : GPSMessage):
        print("i'm gpsCB")
        if msg.status == 0: 
            # GPS 상태 불량
            self.is_gps = False
            print("gps 상태불량")

        else:
            new_xy = self.proj_UTM(msg.longitude, msg.latitude)
            # 이전 좌표와의 차이를 계산
            if hasattr(self, 'prev_xy'):
                dx = new_xy[0] - self.prev_xy[0]
                dy = new_xy[1] - self.prev_xy[1]
                distance = np.sqrt(dx**2 + dy**2)
                # 0.2 m 이상 차이가 날 경우에만 heading 업데이트
                if distance >= 0.2:
                    # arctan2 함수를 이용해 heading 계산 (라디안 -> 도 변환)
                    self.status_msg.heading = np.degrees(np.arctan2(dy, dx))
                    self.prev_xy = new_xy
            else:
                # 첫번째 GPS 메시지인 경우 이전 좌표 저장
                self.prev_xy = new_xy

            self.xy_zone = new_xy

            self.tf_broadcaster.sendTransform((0, 0, 1.2),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "gps",
                            "base_link")
            
            self.tf_broadcaster.sendTransform((1.20, 0, 0.20),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "velodyne",
                            "base_link")
            
            self.is_gps = True
            print("is gps true 로 변환")

    def publishCtrlCmd(self, accel_msg, steering_msg, brake_msg):
        self.ctrl_cmd_msg.accel = accel_msg
        self.ctrl_cmd_msg.steering = steering_msg
        self.ctrl_cmd_msg.brake = brake_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def cornerController(self, corner_theta_degree):
        if corner_theta_degree > 50:
            corner_theta_degree = 50

        target_velocity = -0.3 * corner_theta_degree + 20

        if target_velocity < 0:
            target_velocity = 5
        
        return target_velocity
    
    def brake(self) :
        self.ctrl_cmd_msg.longlCmdType = 1
        self.accel_msg = 0.0
        self.steering_msg = 0.0
        self.brake_msg = 1.0

        self.publishCtrlCmd(self.accel_msg, self.steering_msg, self.brake_msg)

    def egoStatusCB(self, msg): 
        print("i'm egoStatusCB")
        self.current_velocity = msg.velocity.x * 3.6
        if self.current_velocity < 0 :
            self.current_velocity = 0
        self.current_wheel_angle = msg.wheel_angle

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

if __name__ == '__main__':
    try:
        pure_pursuit_= PurePursuit()
    except rospy.ROSInterruptException:
        pass