#!/usr/bin/env python

from time import sleep
import rospy
import tf
import math as m
import numpy as np
from stanley import Stanley
from state import State
from path_selector import PathSelector
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse


max_steer = rospy.get_param("/max_steer", 30.0)  # DEG
desired_speed = rospy.get_param("/desired_speed", 5.)


class StanleyController(object):
    def __init__(self, state):
        self.state = state
        self.stanley = Stanley()
        self.selector = PathSelector(self.state)

        self.response = rospy.Subscriber(
            "/list_Path", PathResponse, callback=self.path_callback)

        self.flag = True
        self.target_idx = 0
        self.path = PathResponse()

        sleep(3.)

        self.selector.makeRequest()
        self.selector.goNext()

        # , _ = self.stanley.calc_target_index(
        #     self.state, self.path.cx, self.path.cy)

    def path_callback(self, msg):
        self.target_idx = 0
        self.path = msg
        self.flag = True

    def makeControlMessage(self):
        if len(self.path.cx) == 0:
            rospy.logwarn("Path error")
            return ControlMessage(0, 0, 0, 0, 0, 0, 0)

        if len(self.path.cx) * 0.97 < self.target_idx and self.flag is True:
            self.selector.makeRequest()
            self.selector.goNext()
            self.flag = False

        di, target_idx = self.stanley.stanley_control(
            self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        # TO DO: You MUST handle control message PERFECTLY.
        # Please remind AorM and Gear

        msg = ControlMessage()

        msg.Speed = desired_speed
        msg.Steer = m.degrees(-di)
        msg.Gear = 2
        msg.brake = 0

        print(msg)

        return msg


if __name__ == "__main__":
    rospy.init_node("stanley_controller")

    state = State(odometry_topic="/odometry/kalman")
    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    controller = StanleyController(state=state)

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        if cmd_pub.get_num_connections() > 0:
            msg = controller.makeControlMessage()
            cmd_pub.publish(msg)

        r.sleep()
