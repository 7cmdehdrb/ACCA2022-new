#!/usr/bin/env python

import math
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


class State(object):
    def __init__(self, odometry_topic="/odometry/kalman"):

        # Subscriber
        self.odom_sub = rospy.Subscriber(
            odometry_topic, Odometry, callback=self.odomCallback)

        # Custum Field
        self.data = Odometry()

        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s
        self.omega = 0.  # rad/s

        # Calculation
        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

    def odomCallback(self, msg):
        self.currentTime = rospy.Time.now()

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        dx = msg.pose.pose.position.x - self.data.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.data.pose.pose.position.y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        dt = (self.currentTime - self.lastTime).to_sec()

        self.v = distance / dt

        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quat)

        self.omega = (yaw - self.yaw) / dt
        self.yaw = yaw

        self.lastTime = self.currentTime

        self.data = msg

    def getArray(self):
        # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        return np.array([self.x, self.y, self.yaw, self.v, self.omega])
