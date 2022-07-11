#!/usr/bin/env python


import rospy
import math as m
import numpy as np
from erp42_control.msg import StanleyError


class SpeedSupporter(object):
    def __init__(self):
        self.stanley_err_sub = rospy.Subscriber(
            "/stanley_error", StanleyError, callback=self.stanleyErrCallback)

        self.h_gain = 10.
        self.c_gain = 5.

        self.hdr = None
        self.ctr = None

        self.p_gain = 1.3
        self.i_gain = 0.
        self.d_gain = 0.

        self.p_err = 0.
        self.i_err = 0.
        self.d_err = 0.

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

        return input_value

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

    def adaptSpeed(self, value, max_value, min_value=5, hdr_threshold=0.01, ctr_threshold=0.08):
        if self.hdr is None or self.ctr is None:
            return value

        hdr = self.func(self.hdr, -3.0, hdr_threshold) * self.h_gain
        ctr = self.func(self.ctr, -5.0, ctr_threshold) * self.c_gain
        err = hdr + ctr

        res = np.clip(value + err, min_value, max_value)

        print(hdr, ctr, res)

        return res
