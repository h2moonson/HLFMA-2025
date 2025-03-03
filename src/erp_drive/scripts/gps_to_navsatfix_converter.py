#!/usr/bin/env python
import rospy
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus

def gps_callback(data):
    navsatfix_msg = NavSatFix()
    navsatfix_msg.header = data.header
    navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
    navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS
    navsatfix_msg.latitude = data.latitude
    navsatfix_msg.longitude = data.longitude
    navsatfix_msg.altitude = data.altitude
    navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    navsatfix_pub.publish(navsatfix_msg)

def listener():
    rospy.init_node('gps_to_navsatfix_converter', anonymous=True)
    rospy.Subscriber("/gps", GPSMessage, gps_callback)
    global navsatfix_pub
    navsatfix_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()
