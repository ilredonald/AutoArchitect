#!/usr/bin/env python3
import rospy
import yaml
import math
import os
import tf

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

# =========================
# PARAMÈTRES
# =========================
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
YAML_FILE = os.path.join(SCRIPT_DIR, "../plan/plan_maison.yaml")

MARGIN = 0.2
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 1.0
DIST_TOL = 0.05

# =========================
# LECTURE DU PLAN
# =========================
with open(YAML_FILE, "r") as f:
    plan = yaml.safe_load(f)

rooms = plan["rooms"]

# =========================
# ROS INIT
# =========================
rospy.init_node("house_tracer_node")

cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
path_pub = rospy.Publisher("/trace_path", Path, queue_size=10)

odom_data = None

def odom_callback(msg):
    global odom_data
    odom_data = msg

rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.sleep(1)

# =========================
# TRACE (PATH)
# =========================
path = Path()
path.header.frame_id = "odom"

def update_trace():
    if odom_data is None:
        return
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    pose.header.stamp = rospy.Time.now()
    pose.pose = odom_data.pose.pose
    path.poses.append(pose)
    path.header.stamp = rospy.Time.now()
    path_pub.publish(path)

# =========================
# OUTILS MOUVEMENT
# =========================
def get_yaw():
    q = odom_data.pose.pose.orientation
    return tf.transformations.euler_from_quaternion(
        [q.x, q.y, q.z, q.w]
    )[2]

def move_to(x_goal, y_goal):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if odom_data is None:
            rate.sleep()
            continue

        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y

        dx = x_goal - x
        dy = y_goal - y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < DIST_TOL:
            break

        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - get_yaw()

        cmd = Twist()
        cmd.linear.x = LINEAR_SPEED
        cmd.angular.z = ANGULAR_SPEED * angle_error

        cmd_pub.publish(cmd)
        update_trace()
        rate.sleep()

    cmd_pub.publish(Twist())

# =========================
# GÉNÉRATION DES MURS
# =========================
def generate_wall_path(room):
    x0 = room["x"] + MARGIN
    y0 = room["y"] + MARGIN
    w = room["width"] - 2*MARGIN
    h = room["height"] - 2*MARGIN

    return [
        (x0, y0),
        (x0 + w, y0),
        (x0 + w, y0 + h),
        (x0, y0 + h),
        (x0, y0)
    ]

# =========================
# CONSTRUCTION DE LA MAISON
# =========================
for room in rooms:
    name = room.get("name", room["type"])
    rospy.loginfo(f"🏗️ Construction : {name}")

    wall_points = generate_wall_path(room)
    for (x, y) in wall_points:
        rospy.loginfo(f"✏️ Tracé vers ({x:.2f}, {y:.2f})")
        move_to(x, y)

rospy.loginfo("🏠 Maison terminée !")
