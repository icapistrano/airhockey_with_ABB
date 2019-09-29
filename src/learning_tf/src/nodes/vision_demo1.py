#!/usr/bin/env python

"vision test showing trajectory"

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

        if table_start <= predict_y <= table_height:
            bounce_trajectory = False
        elif predict_y < table_start:
            table_side = table_start
        elif predict_y > table_height:
            table_side = table_height

    return predict_y

#  LOGICS
def delete_lists():
    #del time_change_in_x[:]
    del puck_x_pos[:]
    del puck_y_pos[:]




rospy.init_node('vision_node')

# Variables
table_start = 0
table_width = 100 # - table starting point
table_height = 50

scale_cm = 0.01 # move in cm

bounce_x = []
bounce_y = []
puck_x_pos = []
puck_y_pos = []


# MAIN
cap = cv2.VideoCapture(0)

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split

while True:
    ret, frame = cap.read()
    flat_perspective_frame = flat_perspective(frame)
    blurred_frame = cv2.GaussianBlur(flat_perspective_frame, (7,7), 0)
    masked_frame = get_mask_frame(blurred_frame)
    resized_frame = cv2.resize(flat_perspective_frame,  (0,0), fx=5, fy=5)

    _, contours, _ = cv2.findContours(masked_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        max_perimeter, max_contour = find_long_contour(contours)
        contour_area = cv2.contourArea(max_contour)
        print contour_area

        if 1 < contour_area < 50:
            x, y, w, h = cv2.boundingRect(max_contour)
            puck_x = x + w / 2
            puck_y = y + h / 2
            print ("x: {}, y: {}".format(puck_x, puck_y))

            puck_x_pos.append(puck_x)
            puck_y_pos.append(puck_y)

            if len(puck_x_pos) == 10:
                if puck_x_pos[0] > puck_x_pos[-1] + 1: # removes doubt of jittering
                    m, b = best_fit_slope_and_intercept(puck_x_pos, puck_y_pos)
                    predict_x = 10
                    predict_y = int((m*predict_x)+b)

                    if table_start <= predict_y <= table_height:
                        cv2.line(flat_perspective_frame, (int(puck_x), int(puck_y)), (predict_x, predict_y), (0,0,255), 2)


                    elif predict_y < table_start:
                        angle = math.degrees(math.atan((puck_x-predict_x)/(puck_y-predict_y)))
                        new_predict_y = predict_bounce_trajectory(table_start, predict_y)
                        cv2.line(flat_perspective_frame, (int(puck_x), int(puck_y)), (predict_x, new_predict_y), (0,0,255), 2)


                    elif predict_y > table_height:
                        angle = math.degrees(math.atan((puck_x-predict_x)/(predict_y-puck_y)))
                        new_predict_y = predict_bounce_trajectory(table_height, predict_y)
                        cv2.line(flat_perspective_frame, (int(puck_x), int(puck_y)), (predict_x, new_predict_y), (0,0,255), 2)

                    else:
                        pass

                delete_lists() # items in list clear every 5 index
            else:
                pass
        else:
            pass
    else:
        pass

    # cv2.imshow("Resized frame", resized_frame)
    cv2.imshow("Camera Frame", frame)
    # cv2.imshow("Masked Frame", masked_frame)
    # cv2.imshow("flat_perspective", flat_perspective_frame)

    bridge = CvBridge()
    ros_frame = bridge.cv2_to_imgmsg(frame, "bgr8")

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows
