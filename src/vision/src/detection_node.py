import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from Detection import Detection

class DetectionNode:
    def __init__(self):
        rospy.init_node("usb_cam_listener", anonymous=True)
    
        self.detection = Detection()
        self.bridge = CvBridge()

        self.detection_pub = rospy.Publisher("/detection_result", Image, queue_size=10)

        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
    
    def img_callback(self, msg: Image):
        img0 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res = self.bridge.cv2_to_imgmsg(self.detection(img0))
        self.detection_pub.publish(res)

if __name__ == "__main__":
    detection_node = DetectionNode()
    rospy.spin()
