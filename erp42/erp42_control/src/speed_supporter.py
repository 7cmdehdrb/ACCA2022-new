#!/usr/bin/env python


import rospy
import math as m
import numpy as np
from erp42_control.msg import StanleyError


class SpeedSupporter(object):
    def __init__(self):
        self.stanley_err_sub = rospy.Subscriber(
            "/stanley_error", StanleyError, callback=self.stanleyErrCallback)

        self.he_gain = rospy.get_param("/stanley_controller/he_gain", 30.0)
        self.ce_gain = rospy.get_param("/stanley_controller/ce_gain", 15.0)

        self.hdr = None
        self.ctr = None

        self.p_gain = rospy.get_param("/stanley_controller/p_gain", 2.0)
        self.i_gain = rospy.get_param("/stanley_controller/i_gain", 0.0)
        self.d_gain = rospy.get_param("/stanley_controller/d_gain", 0.0)

        self.p_err = 0.
        self.i_err = 0.
        self.d_err = 0.

        self.hdr_threshold = rospy.get_param(
            "/stanley_controller/hdr_threshold", 0.01)
        self.ctr_threshold = rospy.get_param(
            "/stanley_controller/ctr_threshold", 0.08)

        self.current = rospy.Time.now()
        self.last = rospy.Time.now()

    def func(self, x, a, b):
        return a * (x - b)

    def stanleyErrCallback(self, msg):
        # almost 0.01 ~ 0.2
        self.hdr = abs(msg.hdr)
        self.ctr = abs(msg.ctr)

    def control(self, current_value, desired_value, max_value, min_value=5):
        desired_value = self.adaptSpeed(
            desired_value, max_value=max_value, min_value=5)
        input_value = self.PIDcontrol(current_value, desired_value)
        input_value = np.clip(input_value, 0., max_value)

        # print(desired_value, input_value)

        return input_value, 0

    def PIDcontrol(self, current_value, desired_value):

        self.current = rospy.Time.now()
        dt = (self.current - self.last).to_sec()

        err = desired_value - current_value
        self.d_err = (err - self.p_err) / dt
        self.p_err = err
        self.i_err += self.p_err * dt

        gain = (self.p_gain * self.p_err) + (self.i_gain *
                                             self.i_err) + (self.d_gain * self.d_err)
        return current_value + gain

    def adaptSpeed(self, value, max_value, min_value=5):
        if self.hdr is None or self.ctr is None:
            return value

        hdr = self.func(self.hdr, -self.he_gain, self.hdr_threshold)
        ctr = self.func(self.ctr, -self.ce_gain, self.ctr_threshold)
        err = hdr + ctr

        res = np.clip(value + err, min_value, max_value)

        # print(hdr, ctr, res)

        return res
