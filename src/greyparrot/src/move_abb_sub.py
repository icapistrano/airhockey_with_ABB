#!/usr/bin/env python

"Script to move ABB to specified waypoints by subscribing to a topic"

import sys
import copy
import rospy
import moveit_msgs
import geometry_msgs
import moveit_commander
from std_msgs.msg import String

home_position = [0.355, 0, 0.3]

def callback(msg):
    # REFERENCED TO home_position
    left_wpose = [[5, 0, 0], [5, 20, 0], [20, 20, 0], [20, -20, 0]]
    right_wpose = [[5, 0, 0], [5, -20, 0], [20, -20, 0], [20, 20, 0]]

    cartesian_planner_execute(home_position[0], home_position[1], home_position[2])

    position = msg.data
    if position == 'left':
        for index, pos in enumerate(left_wpose):
            print ("Going to position {}: {}".format(index, pos))
            cartesian_planner_execute(pos[0], pos[1], pos[2])
            # rospy.sleep(1)

    elif position == 'right':
        for index, pos in enumerate(right_wpose):
            print ("Going to position {}: {}".format(index, pos))
            cartesian_planner_execute(pos[0], pos[1], pos[2])
            # rospy.sleep(1)

    print ""    
    all_zero_pose()

# INITIALIZE
rospy.init_node('simple_abb_sub')
sub = rospy.Subscriber('user_input', String, callback)

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

def cartesian_planner_execute(pos_x, pos_y, pos_z): # function for plannng & executing waypoints
    scale_cm = 0.01 # move in cm
    waypoints = []

    # SYNCHRONIZED MOVEMENTS
    wpose = group.get_current_pose().pose
    wpose.position.x = home_position[0] + pos_x * scale_cm
    wpose.position.y = home_position[1] - pos_y * scale_cm
    wpose.position.z = home_position[2] + pos_z * scale_cm
    waypoints.append(copy.deepcopy(wpose))

    # GENERATES PLAN
    plan, fraction = group.compute_cartesian_path(waypoints, 0.10, 0.0) # move in 10cm resolution

    # MAKE NEW TRAJECTORY WITH LATEST STARTING POSITION
    current_state = robot.get_current_state()
    starting_joint_values = current_state.joint_state.position
    new_trajectory = moveit_msgs.msg.RobotTrajectory()
    new_trajectory.joint_trajectory = plan.joint_trajectory
    new_trajectory.joint_trajectory.points[0].positions = starting_joint_values

    group.execute(new_trajectory, wait=True)

def all_zero_pose():
    joint_goal = group.get_current_joint_values()
    for joint in range(len(joint_goal)):
        joint_goal[joint] = 0

    group.go(joint_goal, wait=False)
    group.stop()

if __name__ == '__main__':
    rospy.spin()
