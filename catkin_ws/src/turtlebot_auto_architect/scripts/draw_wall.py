#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node("draw_wall")
pub = rospy.Publisher("/wall_marker", Marker, queue_size=10)

marker = Marker()
marker.header.frame_id = "map"
marker.type = Marker.POINTS
marker.scale.x = 0.05
marker.scale.y = 0.05
marker.color.a = 1.0
marker.color.r = 1.0

rate = rospy.Rate(5)

while not rospy.is_shutdown():
    p = Point()
    p.x = rospy.get_time() % 2
    p.y = 0
    marker.points.append(p)
    pub.publish(marker)
    rate.sleep()
