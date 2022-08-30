#!/usr/bin/env python

import rospy
import numpy as np
from erp42_control.msg import StanleyError


"""

Export module. Stanley Control Class.
Input: (state(class: State), [cx], [cy], [cyaw], last_target_idx)
Output: steer

"""


class Stanley(object):
    def __init__(self):
        self.__L = 1.040  # [m] Wheel base of vehicle
        self.__k = rospy.get_param(
            "/stanley_controller/c_gain", 0.1)  # control gain
        self.__hdr_ratio = rospy.get_param(
            "/stanley_controller/hdr_ratio", 1.0)

        self.__ind = 0

        self.__hdr = 0.0
        self.__ctr = 0.0

        self.__error_publisher = rospy.Publisher(
            "stanley_error", StanleyError, queue_size=1)

    def resetIdx(self):
        self.__ind = 0

    def setCGain(self, value):
        self.__k = value
        print("SET C GAIN")
        return self.__k

    def setHdrRatio(self, value):
        self.__hdr_ratio = value
        return self.__hdr_ratio

    def stanley_control(self, state, cx, cy, cyaw, last_target_idx, reverse=False):
        """
        Stanley steering control.
        :param state: (State object) => yaw and v
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """

        current_target_idx, error_front_axle = self.calc_target_index(
            state, cx, cy, reverse=reverse)

        # print(current_target_idx)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = (self.normalize_angle(
            cyaw[current_target_idx] - (state.yaw + (np.pi if reverse else 0.)))) * self.__hdr_ratio

        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.__k * error_front_axle,
                             state.v) * (-1.0 if reverse else 1.0)

        # Field
        self.__hdr = theta_e
        self.__ctr = theta_d

        # Steering control
        delta = theta_e + theta_d

        if self.__error_publisher.get_num_connections() > 0:
            msg = StanleyError()
            msg.hdr = self.__hdr
            msg.ctr = self.__ctr
            self.__error_publisher.publish(msg)

        self.__ind = current_target_idx

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

    def calc_target_index(self, state, cx, cy, reverse=False):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position

        fx = state.x + self.__L * \
            np.cos(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)
        fy = state.y + self.__L * \
            np.sin(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]

        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -
                          np.sin(state.yaw + np.pi / 2)]

        error_front_axle = np.dot(
            [dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
