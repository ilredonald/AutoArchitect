#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from stable_baselines3 import PPO
import tf

class RLTracer:

    def __init__(self):
        rospy.init_node("rl_wall_tracer")

        # Charger le modèle PPO
        self.model = PPO.load("wall_tracer_ppo")

        # Publisher cmd_vel
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Publisher pour le tracé
        self.path_pub = rospy.Publisher("/trace_path", Path, queue_size=10)

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        # Variables internes
        self.distance_wall = 1.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.odom_pose = None

        # Path pour visualiser le tracé
        self.path = Path()
        self.path.header.frame_id = "odom"

    # =========================
    # CALLBACKS
    # =========================
    def scan_cb(self, msg):
        # Distance moyenne au mur droit
        right_ranges = msg.ranges[270:300]
        self.distance_wall = float(np.mean(right_ranges))

    def odom_cb(self, msg):
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z
        self.odom_pose = msg.pose.pose
        self.update_trace()

    # =========================
    # TRACE PATH
    # =========================
    def update_trace(self):
        if self.odom_pose is None:
            return
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time.now()
        pose.pose = self.odom_pose
        self.path.poses.append(pose)
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

    # =========================
    # BOUCLE RL
    # =========================
    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Observation pour PPO
            obs = np.array([
                self.distance_wall,
                0.0,  # angle simplifié (à améliorer si besoin)
                self.linear_vel,
                self.angular_vel
            ], dtype=np.float32)

            # Prédiction de l'action
            action, _ = self.model.predict(obs, deterministic=True)

            # Publier la commande
            cmd = Twist()
            cmd.linear.x = float(action[0])
            cmd.angular.z = float(action[1])
            self.cmd_pub.publish(cmd)

            # Collision simple
            if self.distance_wall < 0.15:
                cmd = Twist()
                self.cmd_pub.publish(cmd)
                rospy.logwarn("⚠️ Collision détectée ! Arrêt du robot.")
                rospy.sleep(0.5)

            rate.sleep()

if __name__ == "__main__":
    RLTracer().run()
