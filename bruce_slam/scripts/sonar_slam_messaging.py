#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from bruce_slam.utils.topics import *
from sonar_oculus.msg import OculusPing, OculusPingUncompressed # importing header messages from the feature extraction node; 
pub = None

def callback(data):
    print(data)

def listener():
    # Initialize the 'listener' node with a unique name
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/dummy/mbe_sonar", Image, callback)
    pub = rospy.Publisher(SONAR_TOPIC_UNCOMPRESSED, OculusPingUncompressed, queue_size=1)
    
    # Keep the node running until it is stopped externally
    rospy.spin()    

if __name__ == '__main__':
    listener()
