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
from path_response_with_type import *
from speed_supporter import SpeedSupporter
from time import sleep


max_steer = rospy.get_param("/stanley_controller/max_steer", 30.0)  # DEG
desired_speed = rospy.get_param("/stanley_controller/desired_speed", 7.)
speed_control_enable = rospy.get_param(
    "/stanley_controller/speed_control_enable", True)


class MissionState(Enum):
    DRIVING = 0
    TRAFFIC = 1
    DELIVERY = 2
    STATIC = 3
    DYNAMIC = 4
    PARKING = 5
    END = 9


class StanleyController(object):
    def __init__(self, state):
        """
            Important Objects for Driving
        """

        # Car State Object. Subscribe odometry and update [x, y, v, theta]
        self.state = state

        # Stanley Controller Object
        self.stanley = Stanley()
        self.target_idx = 0

        # Speed Supporter Object. Accelerate in straight track, Decelerate in corner track
        self.supporter = SpeedSupporter()

        # Path Selector. Use path length and stanley's target idx, request path to DB. And then, update self.path
        self.selector = PathSelector(self.state)
        self.path_response = rospy.Subscriber(
            "/path_response", PathResponse, callback=self.path_callback)
        self.path = PathResponse()

        """
            Fields
        """

        # Mission State for state control
        self.mission_state = MissionState.DRIVING
        # Time to prevent request next math without delay
        self.request_time = rospy.Time.now()

        sleep(5.)   # delay

        # Start!
        self.selector.makeRequest()

        # currnet path is not end
        if self.selector.path.end.is_end is True:
            # next path's end point may have traffic sign
            self.mission_state = MissionState.TRAFFIC

        else:
            # Ignore traffic sign
            self.mission_state = MissionState.DRIVING

        self.selector.goNext()

    def path_callback(self, msg):
        # When path response is accepted, reset target idx and update path
        # Upcasting to PathResponseWithType
        self.target_idx = 0
        self.path = PathResponseWithType(msg.start, msg.end,
                                         msg.path_id, list(msg.cx), list(msg.cy), list(msg.cyaw))

    def switchPath(self):
        global desired_speed

        # When current path is almost end
        if len(self.path.cx) * 0.95 < self.target_idx:

            current_time = rospy.Time.now()
            dt = (current_time - self.request_time).to_sec()

            # prevent duplicated request
            if dt > 1:

                # not end
                if self.mission_state != MissionState.END:
                    self.request_time = rospy.Time.now()
                    self.selector.makeRequest()

                    # currnet path is not end
                    if self.selector.path.end.is_end is True:
                        # next path's end point may have traffic sign
                        self.mission_state = MissionState.TRAFFIC

                    else:
                        # Ignore traffic sign
                        self.mission_state = MissionState.DRIVING

                    # next path is end
                    if self.selector.goNext() is None:
                        self.mission_state = MissionState.END

                # end
                else:
                    rospy.loginfo("PATH END")
                    desired_speed = 0

    def drivingControl(self):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)
        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        msg = ControlMessage()

        if speed_control_enable is True:
            speed = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                           desired_value=desired_speed, max_value=int(desired_speed + 2), min_value=5)
        else:
            speed = desired_speed

        msg.Speed = int(speed)
        msg.Steer = m.degrees(-di)
        msg.Gear = 2
        msg.brake = 0

        return msg

    def trafficControl(self):
        msg = self.drivingControl()

        if len(self.path.cx) * 0.9 < self.target_idx:
            # almost end point

            msg.Speed = int(msg.Speed * 0.8)

            if self.path.type == PathType.STRAIGHT:
                pass

            elif self.path.type == PathType.RIGHT:
                pass

            elif self.path.type == PathType.LEFT:
                pass

            else:   # Nonetype
                pass

        return msg

    def makeControlMessage(self):
        # Exception
        if len(self.path.cx) == 0:
            rospy.logwarn("No Path Data...")
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        # Try Update Path
        self.switchPath()

        print(self.mission_state)

        # Switch
        if self.mission_state == MissionState.DRIVING:
            msg = self.drivingControl()

        elif self.mission_state == MissionState.TRAFFIC:
            msg = self.trafficControl()

        elif self.mission_state == MissionState.DELIVERY:
            pass

        elif self.mission_state == MissionState.STATIC:
            pass

        elif self.mission_state == MissionState.DYNAMIC:
            pass

        else:
            rospy.logfatal("Invalide Mission State..!")

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
