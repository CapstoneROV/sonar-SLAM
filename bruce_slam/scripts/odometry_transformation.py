#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf
import tf2_ros
import tf2_geometry_msgs
from bruce_slam.utils.topics import *
odom_pub = rospy.Publisher(LOCALIZATION_ODOM_TOPIC, Odometry, queue_size=1)
# Rotate pose
def rotate_pose(pose, r, p, y):
    rot_tf = TransformStamped()
    rot_tf.transform.translation.x = 0.0
    rot_tf.transform.translation.y = 0.0
    rot_tf.transform.translation.z = 0.0
    quat = tf.transformations.quaternion_from_euler(r, p, y)
    rot_tf.transform.rotation.x = quat[0]
    rot_tf.transform.rotation.y = quat[1]
    rot_tf.transform.rotation.z = quat[2]
    rot_tf.transform.rotation.w = quat[3]
    return tf2_geometry_msgs.do_transform_pose(pose, rot_tf)
# Rotate pose orientation without affecting position
def rotate_pose_orientation(pose, r, p, y):
    quat = tf.transformations.quaternion_from_euler(r, p, y)
    new_pose = PoseStamped()
    new_pose.pose.position.x = 0
    new_pose.pose.position.y = 0
    new_pose.pose.position.z = 0
    new_pose.pose.orientation.x = quat[0]
    new_pose.pose.orientation.y = quat[1]
    new_pose.pose.orientation.z = quat[2]
    new_pose.pose.orientation.w = quat[3]
    val = [
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ]
    # https://robotics.stackexchange.com/a/72843
    # http://docs.ros.org/en/lunar/api/tf/html/python/transformations.html#examples
    pose_tf_matrix = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix((val[0], val[1], val[2])),
        tf.transformations.quaternion_matrix((val[3], val[4], val[5], val[6])),
    )
    _, _, angles, trans, _ = tf.transformations.decompose_matrix(pose_tf_matrix)
    (r, p, y) = angles
    quat = tf.transformations.quaternion_from_euler(r, p, y)
    inv_pose_tf = TransformStamped()
    inv_pose_tf.transform.translation.x = trans[0]
    inv_pose_tf.transform.translation.y = trans[1]
    inv_pose_tf.transform.translation.z = trans[2]
    inv_pose_tf.transform.rotation.x = quat[0]
    inv_pose_tf.transform.rotation.y = quat[1]
    inv_pose_tf.transform.rotation.z = quat[2]
    inv_pose_tf.transform.rotation.w = quat[3]
    return tf2_geometry_msgs.do_transform_pose(new_pose, inv_pose_tf)
def rotate_pose_position(pose, r, p, y):
    rot_tf = TransformStamped()
    rot_tf.transform.translation.x = 0.0
    rot_tf.transform.translation.y = 0.0
    rot_tf.transform.translation.z = 0.0
    quat = tf.transformations.quaternion_from_euler(r, p, y)
    rot_tf.transform.rotation.x = quat[0]
    rot_tf.transform.rotation.y = quat[1]
    rot_tf.transform.rotation.z = quat[2]
    rot_tf.transform.rotation.w = quat[3]
    pos_pose = PoseStamped()
    pos_pose.pose.position = pose.pose.position
    pos_pose.pose.orientation.x = 0
    pos_pose.pose.orientation.y = 0
    pos_pose.pose.orientation.z = 0
    pos_pose.pose.orientation.w = 0
    new_pos_pose = tf2_geometry_msgs.do_transform_pose(pos_pose, rot_tf)
    new_pose = PoseStamped()
    new_pose.pose.orientation = pose.pose.orientation
    new_pose.pose.position = new_pos_pose.pose.position
    return new_pose
def odom_callback(data):
    new_msg = Odometry()
    new_msg.header = data.header
    new_msg.header.frame_id = "odom"
    new_msg.child_frame_id = "base_link"
    dummy_pose = PoseStamped()
    dummy_pose.pose = data.pose.pose
    dummy_pose = rotate_pose_orientation(
        rotate_pose(dummy_pose, 3.14159265359, 0, 0),
        -1.57079632679, 0.0, 0.0)
    new_msg.pose.pose = dummy_pose.pose
    odom_pub.publish(new_msg)
def pose_callback(data):
    pose = rotate_pose_orientation(
        rotate_pose(data, 3.14159265359, 0, 0), 
        -1.57079632679, 0.0, 0.0)
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = pose.pose.position.x
    t.transform.translation.y = pose.pose.position.y
    t.transform.translation.z = pose.pose.position.z
    t.transform.rotation.x = pose.pose.orientation.x
    t.transform.rotation.y = pose.pose.orientation.y
    t.transform.rotation.z = pose.pose.orientation.z
    t.transform.rotation.w = pose.pose.orientation.w
    br.sendTransform(t)
def listener():
    # Initialize the 'listener' node with a unique name
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(MAVROS_ODOM_TOPIC, Odometry, odom_callback)
    rospy.Subscriber(MAVROS_POSE_TOPIC, PoseStamped, pose_callback)
    # Keep the node running until it is stopped externally
    rospy.spin()    
if __name__ == '__main__':
    listener()
