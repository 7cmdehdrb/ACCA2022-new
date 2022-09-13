#!/usr/bin/env python

import sys
import os
import rospy
import rospkg
import tf
import math as m
import numpy as np
from enum import Enum
from time import sleep
# msgs
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse
from lidar_camera_calibration.msg import Signmsg
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import PoseStamped
from mission.msg import obTF

try:
    sys.path.append(rospkg.RosPack().get_path("erp42_control") + "/src")
    from speed_supporter import SpeedSupporter
    from stanley import Stanley
    from state import State, OdomState
    from path_selector import PathSelector, PathType, MissionState

except Exception as ex:
    rospy.logfatal(ex)
    rospy.logfatal("Import Error : State Machine - erp42_control")

try:
    sys.path.append(rospkg.RosPack().get_path("mission") + "/src")
    from parking_final import Parking
    from sign_search import SignSearch
    from obstacle_final import obstacle
    from dynamic_ob import Lidar
    from deliveryAB import Delivery

except Exception as ex:
    rospy.logfatal(ex)
    rospy.logfatal("Import Error : State Machine - mission")


max_steer = rospy.get_param("/state_machine/max_steer", 25.0)  # DEG
speed_control_enable = rospy.get_param(
    "/state_machine/speed_control_enable", True)


def wait_for_stop(duration):
    global current_time, last_time, r, cmd_pub

    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()

        if dt > duration:
            last_time = current_time
            return 0

        cmd_pub.publish(msg)
        r.sleep()


class StateMachine(object):
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

        # Traffic Sign
        self.traffic = SignSearch()

        # Delivery
        self.delivery = Delivery()

        # Dynamic
        self.dynamic = Lidar()

        # Parking
        self.parking = Parking()

        # Static
        self.static = obstacle()

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

    def path_callback(self, msg):
        # When path response is accepted, reset target idx and update path
        self.target_idx = 0
        self.path = msg

    def switchPath(self):
        # When current path is almost end
        if len(self.path.cx) - 20 < self.target_idx:

            rospy.logwarn("Try to change path...!")

            current_time = rospy.Time.now()
            dt = (current_time - self.request_time).to_sec()

            # prevent duplicated request
            if dt > 1:
                # not end
                if self.selector.path.next is not None:
                    self.request_time = rospy.Time.now()
                    self.selector.goNext()
                    self.selector.makeRequest()

                    # next path is end
                    if self.selector.path.next is None:
                        self.mission_state = MissionState.END
                        rospy.loginfo("Missin State : End")

                    # currnet path is not end
                    else:
                        self.mission_state = self.selector.path.mission_type
                        rospy.loginfo("Mission State : %s" %
                                      str(self.mission_state))

                self.target_idx = 0

    def mainControl(self, desired_speed=0):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)

            self.target_idx = target_idx

            desired_speed = self.selector.path.desired_speed if desired_speed == 0 else desired_speed
            di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

            speed, brake = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                                  desired_value=desired_speed, max_value=int(desired_speed + 2), min_value=5)

            return ControlMessage(
                0, 0, 2, int(speed), m.degrees(-di), brake, 0
            )

        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

    def trafficControl(self):
        desired_speed = self.selector.path.desired_speed

        if len(self.path.cx) * 0.8 < self.target_idx:
            # Almost in front of traffic sign
            desired_speed *= 0.5

        elif len(self.path.cx) - 20 < self.target_idx:
            # Stop if required

            try:
                if self.selector.path.path_type == PathType.STRAIGHT:
                    if self.traffic.msg.straight == 0:
                        raise Exception()

                elif self.selector.path.path_type == PathType.LEFT or self.selector.path.path_type == PathType.UTURN:
                    if self.traffic.msg.left == 0:
                        raise Exception()

                elif self.selector.path.path_type == PathType.RIGHT:
                    # ?
                    pass

                else:
                    rospy.logfatal("Invalid Path Type!")
                    raise Exception()

            except Exception:
                # Stop
                return ControlMessage(0, 0, 2, 0, 0, 120, 0)

        msg = self.mainControl(desired_speed=desired_speed)
        return msg

    def deliveryControl(self):
        desired_speed = self.selector.path.desired_speed

        # between delivery sign and erp distance
        dis = np.hypot([self.state.x - self.delivery.panel_x],
                       [self.state.y - self.delivery.panel_y])

        if dis < 10.0:
            # Move to panel
            if self.delivery.is_delivery_path is True:
                desired_speed *= 0.8
                msg = self.mainControl(desired_speed=desired_speed)

                if self.target_idx >= self.delivery.target_idx:
                    wait_for_stop(5)

                    self.request_time = rospy.Time.now()
                    self.selector.goNext()
                    self.selector.makeRequest()
                    self.mission_state = MissionState.DRIVING

            else:
                self.request_time = rospy.Time.now()
                self.selector.goNext()
                self.selector.makeRequest()

                dists = []
                for i in range(len(self.path.cx)):
                    dist = np.hypot(
                        self.path.cx[i] - self.delivery.panel_x, self.path.cy[i] - self.delivery.panel_y)
                    dists.append(dist)

                dists = np.array(dists)
                self.delivery.target_idx = np.argmin(dists)
                self.delivery.is_delivery_path = True

                msg = self.mainControl(desired_speed=desired_speed)

        else:
            # Keep tracking main path
            msg = self.mainControl(desired_speed=desired_speed)

        return msg

    def staticControl(self):
        desired_speed = self.selector.path.desired_speed
        msg = self.mainControl(desired_speed=desired_speed)

        self.static.main()

        if len(self.static.obstacle) != 0:
            return self.static.msg

        return msg

    def dynamicControl(self):
        desired_speed = self.selector.path.desired_speed
        msg = self.mainControl(desired_speed=desired_speed)

        self.dynamic.main()

        if self.dynamic.partTF.front_left == 1 or self.dynamic.partTF.front_right == 1:
            rospy.loginfo("obstacle exist : STOP!")
            return ControlMessage(0, 0, 2, 0, 0, 150, 0)

        return msg

    def parkingControl(self):
        # WTF
        self.parking.main()

        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)
        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        desired_speed = self.selector.path.desired_speed

        if not self.parking.parking_state == 'complete':
            rospy.loginfo("parking")
            return self.parking.msg

        else:
            rospy.loginfo("parking complete!")
            speed, brake = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                                  desired_value=desired_speed, max_value=int(desired_speed + 2), min_value=5)
            msg = ControlMessage()
            msg.Speed = int(speed)
            msg.Steer = m.degrees(-di)
            msg.Gear = 2
            msg.brake = brake

            if self.selector.path.end.is_end is True:
                self.mission_state = MissionState.TRAFFIC
            else:
                self.mission_state == MissionState.DRIVING
            return msg

    def endControl(self):
        if len(self.path.cx) * 0.9 < self.target_idx:
            return ControlMessage(0, 0, 2, 0, 0, 0, 0)

        return self.mainControl()

    def makeControlMessage(self):
        # Exception
        if len(self.path.cx) == 0:
            rospy.logwarn("No Path Data...")
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        # Try Update Path
        self.switchPath()

        # Switch
        if self.mission_state == MissionState.DRIVING:
            msg = self.mainControl()

        elif self.mission_state == MissionState.TRAFFIC:
            msg = self.trafficControl()

        elif self.mission_state == MissionState.DELIVERY:
            msg = self.deliveryControl()

        elif self.mission_state == MissionState.STATIC:
            msg = self.staticControl()

        elif self.mission_state == MissionState.DYNAMIC:
            msg = self.dynamicControl()

        elif self.mission_state == MissionState.PARKING:
            msg = self.parkingControl()

        elif self.mission_state == MissionState.END:
            msg = self.endControl()

        else:
            rospy.logfatal("Invalide Mission State!")
            msg = ControlMessage(0, 0, 2, 3, 0, 0, 0)

        return msg


if __name__ == "__main__":
    rospy.init_node("stanley_controller")

    # state = State(odometry_topic="/odometry/kalman")
    state = OdomState(odometry_topic="/odometry/kalman")

    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    last_time = rospy.Time.now()
    current_time = rospy.Time.now()

    controller = StateMachine(state=state)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if cmd_pub.get_num_connections() > 0 or True:
            msg = controller.makeControlMessage()
            cmd_pub.publish(msg)
            print(controller.mission_state)
            print(msg)
        r.sleep()
