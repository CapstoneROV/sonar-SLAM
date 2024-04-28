#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from bruce_slam.utils.topics import *
from sonar_oculus.msg import OculusFire, OculusPingUncompressed # importing header messages from the feature extraction node; 
sonar_pub = rospy.Publisher(SONAR_TOPIC_UNCOMPRESSED, OculusPingUncompressed, queue_size=1)
debug_pub = rospy.Publisher("debug", Image, queue_size=1)

def callback(data):
# Convert ROS Image to OpenCV format
    bridge = CvBridge()

    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="mono8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return
    
    # Manipulate the image in OpenCV (flip it horizontally)
    cv_image = cv2.flip(cv_image, 1)

    # Convert the manipulated image back to a ROS Image message
    try:
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    new_msg = OculusPingUncompressed()
    new_msg.header = data.header
    new_msg.fire_msg = OculusFire()

    new_msg.ping_id = data.header.seq
    new_msg.part_number = 0
    new_msg.start_time = 0 # rospy.Time.now()

    new_msg.bearings = np.linspace(-4500, 4500, 256).tolist()
    new_msg.num_ranges = 512
    new_msg.range_resolution = 40.0 / new_msg.num_ranges
    new_msg.num_beams = 256

    new_msg.ping = ros_image
    sonar_pub.publish(new_msg)

def listener():
    # Initialize the 'listener' node with a unique name
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(RECT_IMAGE_TOPIC, Image, callback)
    
    # Keep the node running until it is stopped externally
    rospy.spin()    

if __name__ == '__main__':
    listener()
