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
        self.p_gain = 2.6
        self.i_gain = 0.5
        self.d_gain = 1.0

        self.p_err = 0.0
        self.i_err = 0.0
        self.d_err = 0.0
        
        self.speed = 0

        self.current = rospy.Time.now()
        self.last = rospy.Time.now()

        rospy.Subscriber(
            "/erp42_feedback", SerialFeedBack, callback=self.erpCallback
        )

        rospy.Subscriber("p_gain", Float32, callback=self.p_callback)
        rospy.Subscriber("i_gain", Float32, callback=self.i_callback)
        rospy.Subscriber("d_gain", Float32, callback=self.d_callback)


    def erpCallback(self, msg):
        self.speed = mps2kph(msg.speed)

    def p_callback(self, msg):
        self.p_gain = msg.data

    def i_callback(self, msg):
        self.i_gain = msg.data

    def d_callback(self, msg):
        self.d_gain = msg.data

    def PIDControl(self, desired_value):
        self.current = rospy.Time.now()
        
        dt = (self.current - self.last).to_sec()
        
        err = desired_value - self.speed

        self.d_err = (err - self.p_err) / dt
        self.p_err = err
        self.i_err += self.p_err * dt * self.i_gain * (0.0 if self.speed == 0 else 1.0)

        self.last = self.current

        speed = self.speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err) + (self.d_gain * self.d_err)
        
        return int(np.clip(speed, 0, 25))


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

        input_speed = pid.PIDControl(desired_value=7)

        last_time = current_time
        
        print(pid.speed, input_speed)

        cmd_pub.publish(
            ControlMessage(
                0, 0, 2, (int(input_speed)), 0, 0, 0
            )
        )

        r.sleep()
