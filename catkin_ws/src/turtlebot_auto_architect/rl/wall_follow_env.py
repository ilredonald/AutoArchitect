#!/usr/bin/env python3
import gym
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gym import spaces

class WallFollowEnv(gym.Env):

    def __init__(self):
        super().__init__()

        #rospy.init_node("wall_follow_env", anonymous=True)

        # =========================
        # ACTION SPACE
        # =========================
        # [v, w]
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.2], dtype=np.float32),
            high=np.array([0.3, 1.2], dtype=np.float32),
            dtype=np.float32
        )

        # =========================
        # OBSERVATION SPACE
        # =========================
        # [distance_wall, angle_error, linear_vel, angular_vel]
        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4,),
            dtype=np.float32
        )

        # =========================
        # ROS PUB / SUB
        # =========================
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        # =========================
        # ÉTAT INTERNE
        # =========================
        self.distance_wall = 1.0
        self.angle_error = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.prev_angular_vel = 0.0

        self.target_dist = 0.4
        self.collision_dist = 0.15

    # =========================
    # CALLBACKS
    # =========================
    def scan_cb(self, msg):
        # Mur à droite
        right = np.array(msg.ranges[270:300])
        front = np.array(msg.ranges[350:] + msg.ranges[:10])

        # Protection contre inf / nan
        right = np.nan_to_num(right, nan=5.0, posinf=5.0, neginf=0.0)
        front = np.nan_to_num(front, nan=5.0, posinf=5.0, neginf=0.0)

        self.distance_wall = np.clip(np.mean(right), 0.0, 5.0)
        front_dist = np.mean(front)
        self.angle_error = np.clip(front_dist - self.distance_wall, -5.0, 5.0)

    def odom_cb(self, msg):
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    # =========================
    # STEP
    # =========================
    def step(self, action):
        v, w = np.clip(action, self.action_space.low, self.action_space.high)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

        rospy.sleep(0.1)

        # Observation normalisée
        obs = np.array([
            self.distance_wall / 5.0,        # distance normalisée
            self.angle_error / 5.0,          # angle normalisé
            self.linear_vel / 0.3,           # vitesse max 0.3 m/s
            self.angular_vel / 1.2           # angular max 1.2 rad/s
        ], dtype=np.float32)
        obs = np.nan_to_num(obs, nan=0.0, posinf=1.0, neginf=-1.0)

        reward = self.compute_reward()
        done = False

        if self.distance_wall < self.collision_dist:
            reward -= 10.0
            done = True

        return obs, reward, done, {}

    # =========================
    # RESET
    # =========================
    def reset(self):
        stop = Twist()
        self.cmd_pub.publish(stop)
        rospy.sleep(0.5)
        self.prev_angular_vel = 0.0
        return np.zeros(4, dtype=np.float32)

    # =========================
    # RÉCOMPENSE
    # =========================
    def compute_reward(self):
        reward = 0.0

        # 1️⃣ Suivi du mur
        dist_error = abs(self.distance_wall - self.target_dist)
        reward += max(0.0, 1.0 - dist_error * 2.5)

        # 2️⃣ Avancer droit
        reward += max(0.0, self.linear_vel) * 0.5

        # 3️⃣ Pénalité zigzag
        zigzag = abs(self.angular_vel - self.prev_angular_vel)
        reward -= zigzag * 0.2

        # 4️⃣ Trop loin du mur
        if self.distance_wall > 1.0:
            reward -= 0.5

        self.prev_angular_vel = self.angular_vel

        return reward
