#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32


"""
Export module. Stanley Control Class.
Input: (state(class: State), [cx], [cy], [cyaw], last_target_idx)
Output: steer
"""


class Stanley(object):
    def __init__(self):
        super(Stanley, self).__init__()

        self.doPublish = False

        self.L = 1.040  # [m] Wheel base of vehicle
        self.k = rospy.get_param("/c_gain", 0.1)  # control gain
        self.hdr_ratio = rospy.get_param("/hdr_ratio", 0.8)

        self.ind = 0

        self.hdr = 0.0
        self.ctr = 0.0

        self.ctr_publisher = rospy.Publisher(
            "stanley_ctr", Float32, queue_size=1)
        self.hdr_publisher = rospy.Publisher(
            "stanley_hdr", Float32, queue_size=1)

    def setCGain(self, value):
        self.k = value
        return self.k

    def setHdrRatio(self, value):
        self.hdr_ratio = value
        return self.hdr_ratio

    def getHDR(self):
        return self.hdr

    def getCTR(self):
        return self.ctr

    def stanley_control(self, state, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.
        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(
            state, cx, cy)

        # print(current_target_idx)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = (self.normalize_angle(
            cyaw[current_target_idx] - state.yaw)) * self.hdr_ratio

        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.k * error_front_axle, state.v)

        # Field
        self.hdr = theta_e
        self.ctr = theta_d

        # Steering control
        delta = theta_e + theta_d

        if self.doPublish is True:

            self.ctr_publisher.publish(theta_e)
            self.hdr_publisher.publish(theta_d)

        self.ind = current_target_idx

        return delta, current_target_idx

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def calc_target_index(self, state, cx, cy):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + self.L * np.cos(state.yaw) / 2.0
        fy = state.y + self.L * np.sin(state.yaw) / 2.0

        # Search nearest point index
        dx = []
        dy = []

        i = 0
        for icx in cx:
            if i < self.ind - 1000:
                dx.append(float("inf"))
            else:
                dx.append(fx - icx)
            i += 1

        i = 0
        for icy in cy:
            if i < self.ind - 1000:
                dy.append(float("inf"))
            else:
                dy.append(fy - icy)
            i += 1

        # dx = [fx - icx for icx in cx]
        # dy = [fy - icy for icy in cy]

        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -
                          np.sin(state.yaw + np.pi / 2)]

        error_front_axle = np.dot(
            [dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
