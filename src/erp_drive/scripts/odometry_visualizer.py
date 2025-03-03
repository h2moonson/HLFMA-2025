#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def odometry_callback(msg):
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header = msg.header
    pose_stamped_msg.pose = msg.pose.pose
    
    pose_publisher.publish(pose_stamped_msg)
    rospy.loginfo("Published PoseStamped: Position (x: {}, y: {}, z: {}))".format(
        pose_stamped_msg.pose.position.x, pose_stamped_msg.pose.position.y, 
        pose_stamped_msg.pose.position.z, pose_stamped_msg.pose.orientation.x, 
        pose_stamped_msg.pose.orientation.y, pose_stamped_msg.pose.orientation.z, 
        pose_stamped_msg.pose.orientation.w))

def listener():
    global pose_publisher

    rospy.init_node('pose_publisher_node', anonymous=True)
    rospy.Subscriber("/odometry/filtered", Odometry, odometry_callback)
    
    pose_publisher = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
