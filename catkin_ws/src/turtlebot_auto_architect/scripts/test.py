#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WallTracer:
    def __init__(self):
        rospy.init_node("wall_tracer")

        self.marker_pub = rospy.Publisher("/wall_trace", Marker, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "walls"
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05  # épaisseur de la trace

        # Couleur (bleu)
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0

    def odom_callback(self, msg):
        p = Point()
        p.x = msg.pose.pose.position.x
        p.y = msg.pose.pose.position.y
        p.z = 0.0

        self.marker.points.append(p)
        self.marker.header.stamp = rospy.Time.now()

        self.marker_pub.publish(self.marker)

if __name__ == "__main__":
    WallTracer()
    rospy.spin()

