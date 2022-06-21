#!/usr/bin/env python

import rospy
import tf
import math as m
import numpy as np
from stanley import Stanley
from state import State
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse


max_steer = rospy.get_param("/max_steer", 30.0)  # DEG
desired_speed = rospy.get_param("/desired_speed", 5.)


class StanleyController(object):
    def __init__(self, state):
        self.state = state
        self.stanley = Stanley()

        self.path = PathResponse()

        self.target_idx, _ = self.stanley.calc_target_index(
            self.state, self.path.cx, self.path.cy)

        self.response = rospy.Subscriber(
            "/Loaded_Path", PathResponse, callback=self.path_callback)

    def path_callback(self, msg):
        self.target_idx = 0
        self.path = msg

    def makeControlMessage(self):
        if len(self.path.cx) == 0 or len(self.path.cx) == self.target_idx:
            rospy.logwarn("Path error")

        di, target_idx = self.stanley.stanley_control(
            self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        # TO DO: You MUST handle control message PERFECTLY.
        # Please remind AorM and Gear

        msg = ControlMessage()

        msg.Speed = desired_speed
        msg.Steer = m.degrees(-di)
        msg.brake = 0

        return msg


if __name__ == "__main__":
    rospy.init_node("stanley_controller")

    state = State(odometry_topic="/odometry/global")
    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    controller = StanleyController(state=state)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        if cmd_pub.get_num_connections() > 0:
            msg = controller.makeControlMessage()
            cmd_pub.publish(msg)

        r.sleep()
