#!/usr/bin/env python

"Script to move ABB to specified waypoints"

import sys
import copy
import rospy
import moveit_msgs
import geometry_msgs
import moveit_commander

# INITIALIZE
rospy.init_node('simple_abb_move')
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

home_position = [0.355, 0, 0.21]

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
    plan, fraction = group.compute_cartesian_path(waypoints, 0.01, 0.0) # move in 1cm resolution

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

def main():
    left_wpose = [[10, 0, 0], [10, 10, 0], [20, 10, 0], [20, -20, 0]] # referenced to /base_link
    right_wpose = [[10, 0, 0], [10, -10, 0], [20, -10, 0], [20, 20, 0]]

    cartesian_planner_execute(home_position[0], home_position[1], home_position[2])

    position = raw_input("go left or right?")

    if position == 'left':
        for pos in left_wpose:
            print pos
            cartesian_planner_execute(pos[0], pos[1], pos[2])
            rospy.sleep(3)

    elif position == 'right':
        for pos in right_wpose:
            print pos
            cartesian_planner_execute(pos[0], pos[1], pos[2])
            rospy.sleep(3)

    else:
        print ("Staying to home position")

    all_zero_pose()


if __name__ == '__main__':
    main()
