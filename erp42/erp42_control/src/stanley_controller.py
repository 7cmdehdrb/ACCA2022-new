#!/usr/bin/env python

import rospy
import tf
import math as m
import numpy as np
from time import sleep
from stanley import Stanley
from state import State
from path_selector import PathSelector
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse
from speed_supporter import SpeedSupporter


max_steer = rospy.get_param("/max_steer", 30.0)  # DEG
desired_speed = rospy.get_param("/desired_speed", 10.)


class StanleyController(object):
    def __init__(self, state):
        self.state = state
        self.stanley = Stanley()
        self.supporter = SpeedSupporter()
        self.selector = PathSelector(self.state)

        self.response = rospy.Subscriber(
            "/list_Path", PathResponse, callback=self.path_callback)

        self.request_time = rospy.Time.now()
        self.is_last = False
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

    def makeControlMessage(self):
        global desired_speed

        if len(self.path.cx) == 0:
            rospy.logwarn("Path error")
            return ControlMessage(0, 0, 0, 0, 0, 0, 0)

        di, target_idx = self.stanley.stanley_control(
            self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        # TO DO: You MUST handle control message PERFECTLY.
        # Please remind AorM and Gear

        current_time = rospy.Time.now()
        dt = (current_time - self.request_time).to_sec()

        if len(self.path.cx) * 0.97 < self.target_idx and dt > 10. and self.is_last is False:
            self.request_time = rospy.Time.now()
            self.selector.makeRequest()

            if self.selector.goNext() is None:
                desired_speed = 0.
                self.is_last = True

        msg = ControlMessage()

        speed = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                       desired_value=desired_speed, max_value=15.)

        # msg.Speed = desired_speed
        msg.Speed = int(speed)
        msg.Steer = m.degrees(-di)
        msg.Gear = 2
        msg.brake = 0

        # print(msg)

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
