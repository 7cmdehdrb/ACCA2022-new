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
from erp42_control.msg import *


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


def kph2mps(value):
    return value * 0.277778


def mps2kph(value):
    return value * 3.6


def erpCallback(msg):
    global speed
    speed = msg.speed


if __name__ == "__main__":
    rospy.init_node("pid_tuner")

    speed = 0.

    erp_sub = rospy.Subscriber(
        "/erp42_feedback", SerialFeedBack, callback=erpCallback
    )
    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    pid = PID()

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(50)
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        input_speed = pid.PIDControl(
            current_value=speed,
            desired_value=10,
            dt=dt
        )

        last_time = current_time

        cmd_pub.publish(
            ControlMessage(
                0, 0, 2, int(mps2kph(input_speed)), 0, 0, 0
            )
        )

        r.sleep()
