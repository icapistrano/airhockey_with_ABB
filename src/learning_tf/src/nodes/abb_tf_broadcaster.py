#!/usr/bin/env python

# creating a fixed tf frame in reference to base_link

import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('abb_fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        # we create a new tf, from the parent "base_link" to the new child "table_frame"
        # the table_frame frame is 0.1 meters offset in x from the base_link frame
        # the table_frame frame is 0.635 meters offset in y from the base_link frame
        # the table_frame frame is 0.2 meters offset in z from the base_link frame
        br.sendTransform((0.3449964685005368, 0.22500337548094287, 0.19499741058703893),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "table_frame",
                         "base_link")
        rate.sleep()
