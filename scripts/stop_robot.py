#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node('turtlebot_supervisor', anonymous=True)
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    input_str = ""

    while input_str != "Q":
        input_str = raw_input("Press anything to send stop. \"Q\" to quit > ")
        cmd_vel_publisher.publish(Twist())