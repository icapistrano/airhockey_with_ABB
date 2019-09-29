#!/usr/bin/env python

"Script to publish string msgs to topic user_input"

import rospy
from std_msgs.msg import String

rospy.init_node('simple_abb_pub')

def main():
    while True:
        pub = rospy.Publisher('user_input', String, queue_size=10)

        position = raw_input('go left or right?')

        if position == 'left':
            pub.publish(position)

        elif position == 'right':
            pub.publish(position)

        else:
            break

if __name__ == '__main__':
    main()
    rospy.spin()
