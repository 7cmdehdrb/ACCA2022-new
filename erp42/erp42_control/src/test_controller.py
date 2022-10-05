#!/usr/bin/env python

import rospy
import math as m
import numpy as np
from std_msgs.msg import *
from erp42_msgs.msg import *
from erp42_control.msg import *
import threading
import keyboard


def keyboard_input():
    global speed, steer

    while True:
        key = keyboard.read_key()

        if key == "w":
            speed += 1

        elif key == "s":
            speed -= 1

        elif key == "a":
            steer -= 1

        elif key == "d":
            steer += 1

        speed = np.clip(speed, -10, 10)
        steer = np.clip(steer, -30, 30)


if __name__ == "__main__":
    rospy.init_node("controller")

    flag = False

    speed = 0
    steer = 0

    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    th = threading.Thread(target=keyboard_input)
    # th.daemon = True
    th.start()

    r = rospy.Rate(50)
    while not rospy.is_shutdown():

        msg = ControlMessage(
            0, 0, 2 if speed > 0 else 0, abs(speed), steer, 0, 0
        )

        cmd_pub.publish(msg)

        r.sleep()
