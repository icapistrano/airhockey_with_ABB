#!/usr/bin/env python

"test script for demo 24/05/19"
from __future__ import division
import cv2
import numpy as np
import math
import rospy
import time
import sys
import copy
import moveit_commander
import moveit_msgs
import geometry_msgs

from statistics import mean
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState

rospy.init_node('creating_trajectory_node')

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

# MOTION CONTROL
def moving_eef():
    joint_goal = group.get_current_joint_values()
    joint_goal[4] = math.pi/2 # rotating wrist joint to make gripper parallel to table

    group.go(joint_goal, wait = True) # actually go to pose
    group.stop() # removes residual movements

def all_zero_pose():
    joint_goal = group.get_current_joint_values()
    print type(joint_goal)
    for joint in range(len(joint_goal)):
        joint_goal[joint] = 0

    group.go(joint_goal, wait=True)
    group.stop()


def cartesian_planner_execute(pos_x, pos_y, pos_z): # function for plannng & executing waypoints
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x = table_frame[0] + pos_x * scale_cm
    wpose.position.y = table_frame[1] - pos_y * scale_cm
    wpose.position.z = table_frame[2] + pos_z * scale_cm
    waypoints.append(copy.deepcopy(wpose))

    plan, fraction = group.compute_cartesian_path(waypoints, 0.5, 0.0)
    return plan

def cartesian_planner_go(pos_x, pos_y, pos_z): # function for plannng & executing waypoints
    wpose = group.get_current_pose().pose
    wpose.position.x = table_frame[0] + (pos_x * scale_cm)
    wpose.position.y = table_frame[1] - (pos_y * scale_cm)
    wpose.position.z = table_frame[2] + (pos_z * scale_cm)

    plan = group.plan()
    return plan, wpose


table_frame = [0.355, 0.25, 0.21] #0.196003372296
scale_cm = 0.01 # move in cm

all_zero_pose()
rospy.sleep(5)
moving_eef()
rospy.sleep(1)



while True:
    #x = int(raw_input("enter x val: "))
    y = int(raw_input("enter y val: "))

    plan = cartesian_planner_execute(10, y, 0)
    group.execute(plan, wait=False)
    rospy.sleep(1)
    group.stop()
