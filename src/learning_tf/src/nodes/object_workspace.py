#!/usr/bin/env python

"test script for demo 24/05/19"
from __future__ import division
import rospy
import time
import sys
import copy
import moveit_commander
import moveit_msgs
import geometry_msgs

from statistics import mean
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node('object_collision')

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

table_frame = [0.340, 0.235, 0.196003372296]
scale_cm = 0.01 # move in cm
table_width = 120 # - table starting point
table_height = 47


"""
p = geometry_msgs.msg.PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = table_frame[0] + (table_width/2 * scale_cm)
p.pose.position.y = table_frame[1] - (table_height/2 * scale_cm)
p.pose.position.z = table_frame[2]
scene.add_box("table", p, (1.2, 0.47, table_frame[2]))
"""

rospy.sleep(1)
if __name__ == '__main__':
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = -0.36
    p.pose.position.z = 0.5

    scene.add_box("wall_right", p, (0.8, 0.1, 1))

    q = geometry_msgs.msg.PoseStamped()
    q.header.frame_id = robot.get_planning_frame()
    q.pose.position.x = 0
    q.pose.position.y = + 0.36
    q.pose.position.z = 0.5

    scene.add_box("wall_left", q, (0.8, 0.1, 1))

    t = geometry_msgs.msg.PoseStamped()
    t.header.frame_id = robot.get_planning_frame()
    t.pose.position.x = (1.27/2) + (table_frame[0]/2)
    t.pose.position.y = 0
    t.pose.position.z = 0 + (table_frame[2]/2)

    scene.add_box("table", t, (1, 0.47, 0.195))
