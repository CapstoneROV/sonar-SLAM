#!/usr/bin/env python  
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from rti_dvl.msg import DVL
from bar30_depth.msg import Depth
from bruce_slam.utils.topics import *


MAIN_FRAME = "imu_link"

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')
    dvl_pub = rospy.Publisher(DVL_TOPIC, DVL, queue_size=1)
    depth_pub = rospy.Publisher(DEPTH_TOPIC, Depth, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)
    prev_trans = None
    prev_t = None
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(MAIN_FRAME, "map", rospy.Time()).transform.translation
            curr_t = rospy.Time.now()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        if prev_trans is not None:
            delta_t = (curr_t - prev_t).to_sec()
            if delta_t > 1e-7:
                velocity = [
                    (prev_trans.x - trans.x) / delta_t,
                    (prev_trans.y - trans.y) / delta_t,
                    (prev_trans.z - trans.z) / delta_t
                ]

                # Publish DVL
                dvl_msg = DVL()
                dvl_msg.header.stamp = curr_t
                dvl_msg.header.frame_id = MAIN_FRAME
                dvl_msg.velocity.x = velocity[0]
                dvl_msg.velocity.y = - velocity[1]
                dvl_msg.velocity.z = - velocity[2]
                dvl_msg.temperature = 0.0
                dvl_msg.altitude = 0.0
                dvl_msg.time = 0.0
                dvl_pub.publish(dvl_msg)

                # Publish depth
                depth_msg = Depth()
                depth_msg.header.stamp = curr_t
                depth_msg.header.frame_id = MAIN_FRAME
                depth_msg.time = 0.0
                depth_msg.pressure_abs = 0.0
                depth_msg.pressure_diff = 0.0
                depth_msg.temperature = 0.0
                depth_msg.depth = 50 + trans.z
                depth_pub.publish(depth_msg)

        prev_trans = trans
        prev_t = curr_t

        # msg = geometry_msgs.msg.Twist()
        # msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        # msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        # turtle_vel.publish(msg)

        rate.sleep()