#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("simple_move")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

rate = rospy.Rate(10)
msg = Twist()

msg.linear.x = 0.2   # avance
msg.angular.z = 0.0 # tout droit

while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()

