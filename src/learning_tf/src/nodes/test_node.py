#!/usr/bin/env python

import tf
import sys
import copy
import math
import rospy
import numpy as np
import moveit_msgs.msg
import moveit_commander
from geometry_msgs.msg import PoseStamped, Quaternion, Pose

rospy.init_node('frame_checker')
tf_listener = tf.TransformListener()

def create_pose_stamped(position_array=[0,0,0], frame_id="/base_link"):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose.position.x = position_array[0]
    pose_stamped.pose.position.y = position_array[1]
    pose_stamped.pose.position.z = position_array[2]

    pose_stamped.header.stamp = rospy.Time.now()
    return pose_stamped

def convert_trans_rot_to_mat(trans, rot):
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)
    return np.dot(trans_mat, rot_mat)

def convert_pose_to_mat(pose):
    trans = [pose.position.x, pose.position.y, pose.position.z]
    rot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    return convert_trans_rot_to_mat(trans, rot)

def convert_mat_to_pose(mat):
    q = tf.transformations.quaternion_from_matrix(mat)
    t = tf.transformations.translation_from_matrix(mat)

    # print ("q: {}, t: {}".format(q, t))
    pose = Pose()
    pose.position.x = t[0]
    pose.position.y = t[1]
    pose.position.z = t[2]

    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose

def apply_transformation_matrix_to_pose(pose, transform_mat):
    new_mat = np.dot(transform_mat, convert_pose_to_mat(pose))
    return convert_mat_to_pose(new_mat)

def magicbox(current_pose_stamped, target_frame="/table_frame"):
    # waiting for trasforms to become available
    tf_listener.waitForTransform(current_pose_stamped.header.frame_id, target_frame, rospy.Time(), rospy.Duration(4.0))

    # get transfrom and matrix
    frame_trans, frame_rot = tf_listener.lookupTransform(target_frame, current_pose_stamped.header.frame_id, rospy.Time())
    transformation_matrix = convert_trans_rot_to_mat(frame_trans, frame_rot)

    # update the frame id and pose
    current_pose_stamped.header.frame_id = target_frame
    current_pose_stamped.pose = apply_transformation_matrix_to_pose(current_pose_stamped.pose, transformation_matrix)

    # rospy.loginfo("rot_mat: {}, shape: {}".format(mat, np.shape(mat)))
    return current_pose_stamped

def print_pose_stamped(pose_stamped, title="pose stamped"):
    rospy.loginfo("printing... {}".format(title))
    rospy.loginfo("header frame = {}".format(pose_stamped.header.frame_id))
    rospy.loginfo("position: x:{}, y:{}, z:{}".format(pose_stamped.pose.position.x, pose_stamped.pose.position.y,pose_stamped.pose.position.z))
    rospy.loginfo("")

def tf_frames():
    tf_listener.waitForTransform("/base_link", "/table_frame", rospy.Time(), rospy.Duration(4.0)) # this will wait for transforms to be published within 4 secs
    trans,rot = tf_listener.lookupTransform("/base_link", "/table_frame", rospy.Time())
    print "========== transform between /base_link and /table_frame: %s" % trans, rot

def test_magicbox():
    test_pose_stamped = create_pose_stamped([0.1, 0.635, 0.2])
    print_pose_stamped(test_pose_stamped, "test_pose_stamped")

    new_pose_stamped = magicbox(test_pose_stamped)
    print_pose_stamped(new_pose_stamped, "new_pose_stamped")

if __name__ == '__main__':
    test_magicbox()

    rospy.loginfo("Done!")
    # rospy.spin()
