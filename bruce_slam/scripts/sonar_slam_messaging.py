#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from bruce_slam.utils.topics import *
from sonar_oculus.msg import OculusFire, OculusPingUncompressed # importing header messages from the feature extraction node; 
sonar_pub = rospy.Publisher(SONAR_TOPIC_UNCOMPRESSED, OculusPingUncompressed, queue_size=1)

def callback(data):
    new_msg = OculusPingUncompressed()
    new_msg.header = data.header
    new_msg.fire_msg = OculusFire()

    new_msg.ping_id = data.header.seq
    new_msg.part_number = 0
    new_msg.start_time = 0 # rospy.Time.now()

    new_msg.bearings = np.linspace(-45, 45, 256).tolist()
    new_msg.range_resolution = 10.0 / 1024
    new_msg.num_ranges = 1024
    new_msg.num_beams = 256

    new_msg.ping = data
    sonar_pub.publish(new_msg)

def listener():
    # Initialize the 'listener' node with a unique name
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/dummy/mbe_sonar_rect", Image, callback)
    
    # Keep the node running until it is stopped externally
    rospy.spin()    

if __name__ == '__main__':
    listener()
