#!/usr/bin/env python

import rospy
import tf
import math as m
import numpy as np
from enum import Enum
from stanley import Stanley
from state import State
from path_selector import PathSelector
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse
from speed_supporter import SpeedSupporter
from time import sleep


max_steer = rospy.get_param("/stanley_controller/max_steer", 30.0)  # DEG
desired_speed = rospy.get_param("/stanley_controller/desired_speed", 7.)
speed_control_enable = rospy.get_param(
    "/stanley_controller/speed_control_enable", True)


class PathType(Enum):
    STRAIGHT = 0
    RIGHT = 1
    LEFT = 2
    NONE = 3


class PathResponseWithType(PathResponse):
    def __init__(self, *args, **kwds):
        super(PathResponseWithType, self).__init__(*args, **kwds)
        self.type = self.checkPathType()

    def checkPathType(self):
        if (self.start[0] == self.end[0]) is False:
            rospy.loginfo("Not Intersection!")
            return PathType.NONE
        else:
            # 0 : None, 1: -3.14 > + 3.14, 2: +3.14 > -3.14
            trig = 0
            average = 0.
            prev_average = 0.

            for i in range(len(self.cx) - 1):

                if trig == 0:
                    if self.cyaw[i] * self.cyaw[i + 1] < 0. and abs(self.cyaw[i + 1]) > 3.13:
                        # 1: -3.14 > + 3.14, 2: +3.14 > -3.14
                        if self.cyaw[i + 1] > 0.:
                            trig = 1
                        else:
                            trig = 2

                if trig == 1:
                    # -3.14 > +3.14
                    self.cyaw[i + 1] -= 2.0 * m.pi
                elif trig == 2:
                    # +3.14 > -3.14
                    self.cyaw[i + 1] += 2.0 * m.pi

                dyaw = self.cyaw[i + 1] - self.cyaw[i]

                alpha = (i) / (i + 1 + 0.0)
                average = alpha * prev_average + (1 - alpha) * dyaw
                prev_average = average

            print(average)

            if abs(average) < 0.003:
                rospy.loginfo("Guess Straight")
                return PathType.STRAIGHT
            elif average < 0.:
                rospy.loginfo("Guess Right")
                return PathType.RIGHT
            else:
                rospy.loginfo("Guess Left")
                return PathType.LEFT


class StanleyController(object):
    def __init__(self, state):
        self.state = state
        self.stanley = Stanley()
        self.supporter = SpeedSupporter()
        self.selector = PathSelector(self.state)

        self.response = rospy.Subscriber(
            "/path_response", PathResponse, callback=self.path_callback)

        self.request_time = rospy.Time.now()
        self.is_end = False
        self.is_last = False
        self.target_idx = 0
        self.path = PathResponse()

        sleep(2.)

        self.selector.makeRequest()
        self.selector.goNext()
        self.is_end = self.selector.path.end.is_end

    def path_callback(self, msg):
        self.target_idx = 0
        self.path = msg
        PathResponseWithType(msg.start, msg.end,
                             msg.path_id, list(msg.cx), list(msg.cy), list(msg.cyaw))

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

        if len(self.path.cx) * 0.95 < self.target_idx and self.is_last is False:

            current_time = rospy.Time.now()
            dt = (current_time - self.request_time).to_sec()

            if dt > 2.:

                self.request_time = rospy.Time.now()
                self.selector.makeRequest()

                if self.selector.goNext() is None:
                    desired_speed = 0.
                    self.is_last = True
                else:
                    self.is_end = self.selector.path.end.is_end

        msg = ControlMessage()

        if speed_control_enable is True:
            speed = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                           desired_value=desired_speed, max_value=int(desired_speed + 2), min_value=5)

        else:
            speed = desired_speed

        msg.Speed = int(speed) if self.is_last is False else 0
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
