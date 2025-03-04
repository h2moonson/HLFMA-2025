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
from decision.scripts.utils import pathReader,findCurrentwaypoint,findLocalPath,purePursuit,rotateLiDAR2GPS, CCW, pidController
from lidar_object_detection.msg import ObjectInfo


import tf
from math import *
import numpy as np
from tabulate import tabulate
import os
import time 


class EgoStatus:
    def __init__(self):
        self.position = Vector3()
        self.heading = 0.0
        self.velocity = Vector3()


class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        

        # Publisher
        
        
        ########################  lattice  ########################
        ########################  lattice  ########################

        # Subscriber

        ### 파라미터 ###

        self.lfd = 0.0
        self.min_distance_from_path = 0.0
        self.distance_from_remove_path = 0.0
        
        self.curve_steering_msg = 0.0
        self.curve_motor_msg = 0.0


        # Class

if __name__ == '__main__':
    try:
        pure_pursuit_= PurePursuit()
    except rospy.ROSInterruptException:
        pass