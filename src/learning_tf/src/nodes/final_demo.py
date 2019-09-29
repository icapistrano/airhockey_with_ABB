#!/usr/bin/env python

"final demonstration"

from __future__ import division
import cv2
import sys
import math
import time
import copy
import rospy
import numpy as np
import moveit_msgs
import geometry_msgs
import moveit_commander
from statistics import mean
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# IMAGE PROCESSING
def flat_perspective(image):
    top_left = (30, 80)
    top_right = (635, 70)
    bottom_left = (30, 423)
    bottom_right = (635, 413)

    cv2.circle(frame, top_left, 1, (0, 0, 255), 2) # red
    cv2.circle(frame, top_right, 1, (255, 0, 0), 2) # blue
    cv2.circle(frame, bottom_left, 1, (0, 255, 0), 2) # green
    cv2.circle(frame, bottom_right, 1, (255, 0, 255), 2) # purple

    pts1 = np.float32([top_left, top_right, bottom_left, bottom_right])
    pts2 = np.float32(((0,0), (table_width, 0), (0, table_height), (table_width, table_height)))
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    flat_image = cv2.warpPerspective(frame, matrix, (table_width, table_height))

    return flat_image

def get_mask_frame(bgr_frame):
    hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 80, 118])
    higher_green = np.array([86, 211, 243])
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

# TRAJECTORY PREDICTION
def best_fit_slope_and_intercept(xs, ys):
    xs = np.array(xs, dtype = np.float64)
    ys = np.array(ys, dtype = np.float64)
    m = ((mean(xs) * mean(ys)) - (mean(xs*ys))) / ((np.square(mean(xs))) - (mean(np.square(xs))))
    b = mean(ys) - (m*mean(xs))

    return m, b

def predict_bounce_trajectory(puck_bounced_on, predict_y):
    bounce_trajectory = True
    table_side = puck_bounced_on
    while bounce_trajectory:
        error = predict_y - table_side
        predict_y = error * -1

        if table_start < predict_y < table_height:
            bounce_trajectory = False
        elif predict_y <= table_start:
            table_side = table_start
        elif predict_y >= table_height:
            table_side = table_height

    return predict_y

# MOTION CONTROL
def moving_eef():
    joint_goal = group.get_current_joint_values()
    joint_goal[4] = math.pi/2 # rotating wrist joint to make gripper parallel to table

    group.go(joint_goal, wait=True) # actually go to pose
    group.stop() # removes residual movements

def all_zero_pose():
    joint_goal = group.get_current_joint_values()
    for joint in range(len(joint_goal)):
        joint_goal[joint] = 0

    group.go(joint_goal, wait=False)
    group.stop()

def cartesian_planner_execute(pos_x, pos_y, pos_z): # function for plannng & executing waypoints
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x = table_frame[0] + pos_x * scale_cm
    wpose.position.y = table_frame[1] - pos_y * scale_cm
    wpose.position.z = table_frame[2] + pos_z * scale_cm

    waypoints.append(copy.deepcopy(wpose))
    plan, fraction = group.compute_cartesian_path(waypoints, 0.5, 0.0)

    current_state = robot.get_current_state()
    starting_joint_values = current_state.joint_state.position

    new_trajectory = moveit_msgs.msg.RobotTrajectory()
    new_trajectory.joint_trajectory = plan.joint_trajectory
    new_trajectory.joint_trajectory.points[0].positions = starting_joint_values

    group.execute(new_trajectory, wait=False)

def constrain_robot():
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = -0.36
    p.pose.position.z = 0.5

    q = geometry_msgs.msg.PoseStamped()
    q.header.frame_id = robot.get_planning_frame()
    q.pose.position.x = 0
    q.pose.position.y = + 0.36
    q.pose.position.z = 0.5

    t = geometry_msgs.msg.PoseStamped()
    t.header.frame_id = robot.get_planning_frame()
    t.pose.position.x = (1.27/2) + (table_frame[0]/2)
    t.pose.position.y = 0
    t.pose.position.z = 0 + (table_frame[2]/2)
    scene.add_box("wall_right", p, (0.8, 0.1, 1))
    scene.add_box("wall_left", q, (0.8, 0.1, 1))
    scene.add_box("table", t, (1, 0.47, 0.195))

# LOGICS
def delete_lists():
    del time_change_in_x[:]
    del puck_xs[:]
    del puck_ys[:]



# INITIALIZE
rospy.init_node('air_hockey_node')
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

# Variables
flag = True
scale_cm = 0.01 # move in cm

table_start = 0
table_width = 100 # - table starting point
table_height = 51

puck_xs = []
puck_ys = []
bounce_x = []
bounce_y = []
time_change_in_x = []

table_frame = [0.355, 0.25, 0.21] #0.196003372296


# MAIN
moving_eef()
rospy.sleep(1)
cartesian_planner_execute(0, table_height/2, 0)
rospy.sleep(1)
# constrain_robot()

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    flat_perspective_frame = flat_perspective(frame)
    blurred_frame = cv2.GaussianBlur(flat_perspective_frame, (3,3), 0)
    masked_frame = get_mask_frame(blurred_frame)
    # resized_frame = cv2.resize(flat_perspective_frame,  (0,0), fx=5, fy=5)

    _, contours, _ = cv2.findContours(masked_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        max_perimeter, max_contour = find_long_contour(contours)
        contour_area = cv2.contourArea(max_contour)

        if 0.1 < contour_area < 50:
            x, y, w, h = cv2.boundingRect(max_contour)
            puck_x = int(x + w / 2)
            puck_y = int(y + h / 2)

            puck_xs.append(puck_x)
            time_change_in_x.append(time.time())
            puck_ys.append(puck_y)

            if len(puck_xs) == 5:
                if puck_xs[0] > puck_xs[-1] + 1 and flag == True: # removes doubt of jittering
                    # counter_start = time.time()
                    m, b = best_fit_slope_and_intercept(puck_xs, puck_ys)
                    predict_x = 10
                    predict_y = int((m*predict_x)+b)
                    # rospy.loginfo("last recorded puck position x: {}, y:{}".format(puck_x, puck_y))
                    # rospy.loginfo("prediction at intercept line: {}".format(predict_y))

                    # prediction_counter = time.time()
                    # print ("prediction took: {} ms".format((prediction_counter-counter_start)*1000))

                    # time_dif_puck = (time_change_in_x[-1] - time_change_in_x[0]) #  measured in seconds
                    # distance_dif_puck =  (puck_xs[0] - puck_xs[-1]) # cm
                    # puck_speed = (distance_dif_puck/time_dif_puck)
                    # time_to_reach = (puck_x - predict_x)/puck_speed
                    #
                    # rospy.loginfo("speed of incoming puck: {} cm/seconds".format(int(puck_speed)))
                    # # rospy.loginfo ("last recorded puck x: {}, puck y: {}".format(puck_xs[-1], puck_ys[-1]))
                    # print ""

                    if table_start < predict_y < table_height:
                        cartesian_planner_execute(predict_x, predict_y, 0)
                        group.stop()
                        rospy.sleep(1)

                    elif predict_y <= table_start:
                        new_predict_y = predict_bounce_trajectory(table_start, predict_y)
                        cartesian_planner_execute(predict_x, new_predict_y, 0)
                        group.stop()
                        rospy.sleep(1)

                        # angle = math.degrees(math.atan((puck_x-predict_x)/(puck_y-predict_y)))
                        # print "bounce on your right side"
                        # print ("angle: {}".format(angle))
                        # print ("opposite: {}".format(puck_x-predict_x))
                        # print ("adjacent: {}".format(puck_y-predict_y))
                        # print ("last recorded puck position x: {}, y: {}".format(puck_x, puck_y))
                        # print ""


                    elif predict_y >= table_height:
                        new_predict_y = predict_bounce_trajectory(table_height, predict_y)
                        cartesian_planner_execute(predict_x, new_predict_y, 0)
                        group.stop()
                        rospy.sleep(1)

                        # angle = math.degrees(math.atan((puck_x-predict_x)/(predict_y-puck_y)))
                        # print "bounce on left side"
                        # print ("angle: {}".format(angle))
                        # print ("opposite: {}".format(puck_x-predict_x))
                        # print ("adjacent: {}".format(predict_y-puck_y))
                        # print ("last recorded puck position x: {}, y: {}".format(puck_x, puck_y))
                        # print ""

                    else:
                        pass

                    cartesian_planner_execute(0, table_height/2, 0)
                    group.stop()
                    rospy.sleep(1)
                    flag = False

                elif puck_xs[0] < puck_xs[-1]: # puck moving towards human
                    flag = True

                delete_lists() # clears list
            else:
                pass
        else:
            pass
    else:
        pass

    # cv2.imshow("Camera Frame", frame)
    cv2.imshow("Masked Frame", masked_frame)
    #cv2.imshow("flat_perspective", flat_perspective_frame)
    #cv2.imshow("Resized frame", resized_frame)

    bridge = CvBridge()
    ros_frame = bridge.cv2_to_imgmsg(frame, "bgr8")

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows

all_zero_pose()
rospy.sleep(1)
moveit_commander.roscpp_shutdown()
