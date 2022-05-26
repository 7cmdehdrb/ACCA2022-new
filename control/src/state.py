#!/usr/bin/env python

import sys
import os
import rospy
import tf
import math as m
import numpy as np
from nav_msgs.msg import Odometry
from erp42_msgs.msg import SerialFeedBack

"""

Export module. State class
Define vihicle's state
{
    x_position,
    y_position,
    heading_angle(yaw),
    velocity,
    x_velocity,
    y_velocity
}

USE odometryCallback to automatically update your vihicle's state

"""


class State(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.data = Odometry()

        rospy.Subscriber("/erp42_encoder", SerialFeedBack,
                         self.encoderCallback)

        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

        self.dt = 0.1

        self.x = x
        self.y = y
        self.yaw = yaw

        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw

        self.v = v
        self.dx = self.v * m.cos(self.yaw)
        self.dy = self.v * m.sin(self.yaw)

    def odometryCallback(self, msg):
        self.data = msg

        self.currentTime = rospy.Time.now()

        self.dt = (self.currentTime - self.lastTime).to_sec()

        if self.dt == 0:
            return

        self.x = self.data.pose.pose.position.x
        self.y = self.data.pose.pose.position.y

        x = self.data.pose.pose.orientation.x
        y = self.data.pose.pose.orientation.y
        z = self.data.pose.pose.orientation.z
        w = self.data.pose.pose.orientation.w

        quaternion_array = [x, y, z, w]

        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion_array)

        self.yaw = yaw

        self.dx = (self.x - self.last_x) / self.dt
        self.dy = (self.x - self.last_y) / self.dt
        self.dyaw = (self.yaw - self.last_yaw) / self.dt
        self.v = np.hypot(self.dx, self.dy)

        self.last_x = self.x
        self.last_y = self.y
        self.last_yaw = self.yaw
        self.lastTime = rospy.Time.now()

    def encoderCallback(self, msg):
        self.v = msg.speed
