#!/usr/bin/env python

import rospy
import tf
import math as m
import numpy as np
from enum import Enum
from stanley import Stanley
from state import State, OdomState
from path_selector import PathSelector
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse
from lidar_camera_calibration.msg import Signmsg
from path_response_with_type import *
from time import sleep


max_steer = rospy.get_param("/stanley_controller/max_steer", 30.0)  # DEG
desired_speed = rospy.get_param("/stanley_controller/desired_speed", 7.)
speed_control_enable = rospy.get_param(
    "/stanley_controller/speed_control_enable", True)


class StanleyController(object):
    def __init__(self, state):
        """
            Important Objects for Driving
        """

        # Car State Object. Subscribe odometry and update [x, y, v, theta]
        self.state = state

        # Stanley Controller Object
        self.stanley = Stanley()
        # self.parameter_tuner = ParameterTuner()
        self.target_idx = 0

        # Path Selector. Use path length and stanley's target idx, request path to DB. And then, update self.path
        self.selector = PathSelector(self.state)
        self.path_response = rospy.Subscriber(
            "/path_response", PathResponse, callback=self.path_callback)
        self.path = PathResponse()

        """
            Fields
        """
        # Time to prevent request next math without delay
        self.request_time = rospy.Time.now()

        sleep(5.)   # delay

        # Start!
        self.selector.makeRequest()
        self.selector.goNext()

    def path_callback(self, msg):
        self.target_idx = 0
        self.path = PathResponse(
            msg.start, msg.end, msg.path_id, msg.cx, msg.cy, list(
                np.array(msg.cyaw) + np.pi)
        )

    def trafficCallback(self, msg):
        self.trafficSign = msg

    def switchPath(self):
        global desired_speed

        # When current path is almost end
        if len(self.path.cx) * 0.95 < self.target_idx:

            current_time = rospy.Time.now()
            dt = (current_time - self.request_time).to_sec()

            # prevent duplicated request
            if dt > 1:
                self.request_time = rospy.Time.now()
                self.selector.makeRequest()
                self.selector.goNext()

    def drivingControl(self):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx, reverse=True)
        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        msg = ControlMessage()

        speed = desired_speed

        msg.Speed = int(speed)
        msg.Steer = m.degrees(di)
        msg.Gear = 1
        msg.brake = 0

        return msg

    def makeControlMessage(self):
        # Exception
        if len(self.path.cx) == 0:
            rospy.logwarn("No Path Data...")
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        # Try Update Path
        self.switchPath()

        msg = self.drivingControl()

        return msg


if __name__ == "__main__":
    rospy.init_node("stanley_controller")

    state = State(odometry_topic="/odometry/kalman")
    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    controller = StanleyController(state=state)

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        if cmd_pub.get_num_connections() > 0:
            msg = controller.makeControlMessage()
            cmd_pub.publish(msg)
            # print(msg)
        r.sleep()
