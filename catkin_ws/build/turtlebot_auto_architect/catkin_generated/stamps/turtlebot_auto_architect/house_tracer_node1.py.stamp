#!/usr/bin/env python3
import rospy
import yaml
import os
import subprocess
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import sys

# =========================
# PARAMÈTRES
# =========================
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
YAML_FILE = os.path.join(SCRIPT_DIR, "../plan/plan_maison.yaml")

TRACE_DURATION_PER_WALL = 4.0  # temps RL par mur (secondes)

# =========================
# LECTURE DU PLAN
# =========================
with open(YAML_FILE, "r") as f:
    plan = yaml.safe_load(f)

rooms = plan["rooms"]

# =========================
# ROS INIT
# =========================
rospy.init_node("house_tracer_node1")

path_pub = rospy.Publisher("/trace_path", Path, queue_size=10)
odom_data = None

def odom_cb(msg):
    global odom_data
    odom_data = msg

rospy.Subscriber("/odom", Odometry, odom_cb)
rospy.sleep(1)

# =========================
# TRACE DU PASSAGE
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
# LANCEMENT DU RL
# =========================
def trace_wall_with_rl(duration):
    rospy.loginfo("🧠 Lancement du contrôleur RL (PPO)")
    
    rl_script = os.path.join(SCRIPT_DIR, "rl/inference_node.py")
    rl_proc = subprocess.Popen([sys.executable, rl_script])


    start = rospy.Time.now()
    rate = rospy.Rate(10)

    while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
        update_trace()
        rate.sleep()

    rl_proc.terminate()
    rospy.loginfo("🛑 Fin du tracé RL")

# =========================
# GÉNÉRATION DES MURS (LOGIQUE)
# =========================
def generate_wall_segments(room):
    x = room["x"]
    y = room["y"]
    w = room["width"]
    h = room["height"]

    return [
        ("bas", x, y),
        ("droite", x + w, y),
        ("haut", x + w, y + h),
        ("gauche", x, y + h),
    ]

# =========================
# CONSTRUCTION DE LA MAISON
# =========================
for room in rooms:
    name = room.get("name", room["type"])
    rospy.loginfo(f"🏗️ Construction de la pièce : {name}")

    walls = generate_wall_segments(room)

    for wall_name, _, _ in walls:
        rospy.loginfo(f"✏️ Tracé du mur : {wall_name}")
        trace_wall_with_rl(TRACE_DURATION_PER_WALL)

rospy.loginfo("🏠 Maison terminée !")
