#!/usr/bin/env python3
import gymnasium as gym
import numpy as np
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gymnasium import spaces

class AutoArchitectEnv(gym.Env):
    def __init__(self):
        rospy.init_node("auto_architect_env", anonymous=True)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        self.position = np.zeros(2)
        self.yaw = 0.0

        self.target = np.array([2.0, 0.0])

        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0]),
            high=np.array([0.22, 1.0]),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(3,),
            dtype=np.float32
        )

    def odom_cb(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z)
        cosy = 1.0 - 2.0 * (q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def step(self, action):
        vel = Twist()
        vel.linear.x = float(action[0])
        vel.angular.z = float(action[1])
        self.cmd_pub.publish(vel)

        rospy.sleep(0.1)

        dist = np.linalg.norm(self.target - self.position)
        angle = math.atan2(
            self.target[1] - self.position[1],
            self.target[0] - self.position[0]
        )
        angle_error = angle - self.yaw

        reward = -dist - abs(angle_error)

        done = dist < 0.1
        if done:
            reward += 10.0

        obs = np.array([dist, angle_error, self.yaw], dtype=np.float32)
        return obs, reward, done, False, {}

    def reset(self, seed=None, options=None):
        return np.zeros(3, dtype=np.float32), {}
