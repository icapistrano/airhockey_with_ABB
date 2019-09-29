#!/usr/bin/env python

"test script for demo 05/04/19"
from __future__ import division
import cv2
import numpy as np
import math
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs
import geometry_msgs

from statistics import mean
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Functions
def flat_perspective(image):
    top_left = (10, 110)
    top_right = (630, 110)
    bottom_left = (10, 380)
    bottom_right = (630, 380)

    cv2.circle(frame, top_left, 1, (0, 0, 255), 5) # red
    cv2.circle(frame, top_right, 1, (255, 0, 0), 5) # blue
    cv2.circle(frame, bottom_left, 1, (0, 255, 0), 5) # green
    cv2.circle(frame, bottom_right, 1, (255, 0, 255), 5) # purple

    pts1 = np.float32([top_left, top_right, bottom_left, bottom_right])
    pts2 = np.float32(((0,0), (table_width, 0), (0, table_height), (table_width, table_height)))

    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    flat_image = cv2.warpPerspective(frame, matrix, (table_width, table_height))
    return flat_image

def get_mask_frame(bgr_frame):
    hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

    lower_green = np.array([56, 98, 82])
    higher_green = np.array([84, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_green, higher_green)

    return mask

def find_long_contour(contours_list):
    contour_perimeter = None
    contour_points = None

    if len(contours_list) > 0:
        for contour in contours_list:
            perimeter = cv2.arcLength(contour, True)
            if perimeter > contour_perimeter:
                contour_perimeter = perimeter
                contour_points = contour
            return contour_perimeter, contour_points

def best_fit_slope_and_intercept(xs, ys):
    x_minus_xs_mean = []
    x_minus_xs_mean_square = []
    y_minus_ys_mean = []
    sum_x_y = []

    for x, y in zip(xs, ys):
        x_minus = x - mean(xs)
        x_minus_xs_mean.append(x_minus)

        x_square = np.square(x_minus)
        x_minus_xs_mean_square.append(x_square)

        y_minus = y - mean(ys)
        y_minus_ys_mean.append(y_minus)

    for x, y in zip(x_minus_xs_mean, y_minus_ys_mean):
        total = x * y
        sum_x_y.append(total)

    m = sum(sum_x_y) / sum(x_minus_xs_mean_square)
    b = mean(ys) - (m * mean(xs))

    return m, b

def moving_eef():
    joint_goal = group.get_current_joint_values()

    joint_goal[4] = math.pi/2 # rotating wrist joint to make gripper parallel to table

    group.go(joint_goal, wait = True) # actually go to pose
    group.stop() # removes residual movements

def go_to_pos(pos_y):
    waypoints = []
    offset = [0.405000445242, 0.225007887688, 0.196003372296]

    wpose = group.get_current_pose().pose
    wpose.position.y = offset[1] - (pos_y * scale_cm)
    waypoints.append(copy.deepcopy(wpose))

    plan, fraction = group.compute_cartesian_path(waypoints, # waypoints to follow
                                                        0.01, # eef_step, moves in cm resolution
                                                        0.0)  #jump_threshold

    group.execute(plan, wait = True)

def cartesian_planner_go(pos_x, pos_y, pos_z): # function for plannng & executing waypoints
    wpose = group.get_current_pose().pose
    wpose.position.x = table_frame[0] + pos_x * scale_cm
    wpose.position.y = table_frame[1] - pos_y * scale_cm
    wpose.position.z = table_frame[2] + pos_z * scale_cm

    start_excecution_time = rospy.get_rostime()
    group.go(wpose, wait = False) # try running this! should run faster, no waypoints just end goal.
    end_excecution_time = rospy.get_rostime()
    print "total time of execution: {}".format(end_excecution_time-start_excecution_time)
    group.clear_pose_targets()


def go_home_position():
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x = 0.405000445242
    wpose.position.y = 0.225007887688
    wpose.position.z = 0.2 # dummy

    #wpose.position.z = 0.196003372296

    waypoints.append(copy.deepcopy(wpose))

    plan, fraction = group.compute_cartesian_path(waypoints, # waypoints to follow
                                                        0.01, # eef_step, moves in cm resolution
                                                        0.0)  #jump_threshold

    group.execute(plan, wait = True)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('follow_puck_demo')

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size = 10)

# Variables
cap = cv2.VideoCapture(1)
table_width = 120 # - table starting point
table_height = 46
scale_cm = 0.01 # move in cm

puck_posy_rate = 20
average_pos = []
weights = []

table_frame = [0.405000445242, 0.225007887688 , 0.196003372296]

for i in range(puck_posy_rate):
    percentage = i/puck_posy_rate * 100
    weights.append(percentage)

moving_eef()
rospy.sleep(1)
go_home_position()

while True:

    ret, frame = cap.read()
    flat_perspective_frame = flat_perspective(frame)
    masked_frame = get_mask_frame(flat_perspective_frame)
    blurred_frame = cv2.GaussianBlur(masked_frame, (15,15), 0)

    _, contours, _ = cv2.findContours(blurred_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(average_pos) == 1: # start counter
        start_time = rospy.get_rostime()

    if len(contours) > 0:
        max_perimeter, max_contour = find_long_contour(contours)
        contour_area = cv2.contourArea(max_contour)

        if 100 < contour_area < 5000:
            x, y, w, h = cv2.boundingRect(max_contour)
            puck_x = x + w / 2
            puck_y = y + h / 2
            average_pos.append(puck_y)

            if len(average_pos) >= puck_posy_rate:
                end_time = rospy.get_rostime()
                total_time = end_time - start_time

                average_y = mean(average_pos)
                weight_average_y = np.average(average_pos, weights = weights)

                #print average_y, weight_average_y
                #break

                #rospy.loginfo("total time of {} pucks:{}".format(len(average_pos), total_time.secs))
                #go_to_pos(int(weight_average_y))
                cartesian_planner_go(0, int(weight_average_y), 0)
                rospy.sleep(0.1)
                del average_pos[:-1]


    else:
        pass


    cv2.imshow("Camera Frame", frame)
    cv2.imshow("Flat Perspective", masked_frame)

    bridge = CvBridge()
    ros_frame = bridge.cv2_to_imgmsg(frame, "bgr8")

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows
