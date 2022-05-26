#!/usr/bin/env python

import rospy
import tf
import math as m
import numpy as np
from stanley import Stanley
from state import State
from erp42_control.msg import ControlMessage


max_steer = rospy.get_param("/max_steer", 30.0)  # DEG
desired_speed = rospy.get_param("/desired_speed", 5.0)


# Test
class Path(object):
    def __init__(self):
        self.start = None
        self.end = None

        self.cx = []
        self.cy = []
        self.cyaw = []


class StanleyController(object):
    def __init__(self, state, path=Path()):
        self.state = state
        self.stanley = Stanley()

        self.target_idx, _ = self.stanley.calc_target_index(
            self.state, self.path.cx, self.path.cy)

    def makeControlMessage(self):
        di, self.target_idx = self.stanley.stanley_control(
            self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        # TO DO: You MUST handle control message PERFECTLY.
        # Please remind AorM and Gear

        msg = ControlMessage()

        msg.Speed = desired_speed
        msg.Steer = -di
        msg.brake = 0

        return msg


if __name__ == "__main__":
    rospy.init_node("stanley_controller")

    state = State(odometry_topic="/odometry/global")
    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    controller = StanleyController(state=state, path=Path())

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():

        if cmd_pub.get_num_connections() > 0:
            msg = controller.makeControlMessage()
            cmd_pub.publish(msg)

        r.sleep()
