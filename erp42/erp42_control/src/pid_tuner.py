#!/usr/bin/env python

import os
import sys
import rospy
import rospkg
import math as m
import numpy as np
from state import OdomState
from std_msgs.msg import *
from erp42_msgs.msg import *


class PID(object):
    def __init__(self):
        self.p_gain = 1.0
        self.i_gain = 0.0
        self.d_gain = 0.0

        self.p_err = 0.0
        self.i_err = 0.0
        self.d_err = 0.0

        rospy.Subscriber("load", Float32, self.load_callback)
        rospy.Subscriber("p_gain", Float32, callback=self.p_callback)
        rospy.Subscriber("i_gain", Float32, callback=self.i_callback)
        rospy.Subscriber("d_gain", Float32, callback=self.d_callback)

    def p_callback(self, msg):
        self.p_gain = msg.data

    def i_callback(self, msg):
        self.i_gain = msg.data

    def d_callback(self, msg):
        self.d_gain = msg.data

    def PIDControl(self, current_value, desired_value, dt):
        err = desired_value - current_value

        self.d_err = (err - self.p_err) / dt
        self.p_err = err
        self.i_err += self.p_err * dt

        return current_value + (self.p_gain * self.p_err) + (self.i_gain *
                                                             self.i_err) + (self.d_gain * self.d_err)


if __name__ == "__main__":
    rospy.init_node("pid_tuner")

    state = OdomState(odometry_topic="/odometry/kalman")
    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)
