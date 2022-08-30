#!/usr/bin/env python

import os
import sys
import rospy
import rospkg
import numpy as np
import math as m
from geometry_msgs.msg import *
from path_plan.msg import PathResponse
from erp42_control.msg import ControlMessage
from cubic_spline_planner import calc_spline_course
from abc import *
from enum import Enum
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
import csv
from nav_msgs.msg import Odometry


try:
    sys.path.append(rospkg.RosPack().get_path("erp42_control") + "/src")
    from stanley import Stanley
    from state import State, OdomState
except Exception as ex:
    rospy.logfatal("Import Error : Vertical Parking")
    rospy.logfatal(ex)

def loadCSV():

    x, y, yaw = [], [], []

    with open('/home/enbang/catkin_ws/src/ACCA2022-new/path_plan/path/center.csv', "r") as csvFile:
        reader = csv.reader(csvFile, delimiter=",")
        for row in reader:
            x.append(row[0])
            y.append(row[1])
            x.append(row[2])
    return x, y, yaw


class ParkingState(Enum):
    '''Searching = 0     # find empty parking lot
    Reset = 1        # go back to start point & path plan
    Parking = 2       # tracking parking path
    Backward =  3     # escape
    End = 4        # end
    Deceleration = 5  # deceleration
'''
    Searching = 0     # find empty parking lot
    Deceleration1 = 1  # deceleration
    Reset = 2         # go back to start point & path plan
    Deceleration2 = 3  # deceleration
    Parking = 4       # tracking parking path
    Deceleration3 = 5  # deceleration
    Backward = 6     # escape
    Deceleration4 = 7  # deceleration
    End = 8           # end


class VerticalParkingBase(object):
    __metaclass__ = ABCMeta

    def __init__(self, state, stanley):
        self.state = state
        self.stanley = stanley
        self.path = PathResponse()
        self.local_path = PathResponse()

        self.startPoint = None

        self.parking_state = ParkingState.Searching

        self.target_idx = 0
        self.brake = 50

    def createPath(self, end_point=Point()):
        # cx, cy, cyaw, _, _ = calc_spline_course([self.state.x, end_point.x], [
        #     self.state.y, end_point.y], ds=0.1)

        x, y, yaw = loadCSV()
        path = PathResponse()
        path.cx = x
        path.cy = y
        path.cyaw = yaw

        return PathResponse()

    def makeControlMessage(self, path):
        di, target_idx = self.stanley.stanley_control(
            state=self.state,
            cx=path.cx,
            cy=path.cy,
            cyaw=path.cyaw,
            last_target_idx=self.target_idx
        )

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(30.0), m.radians(30.0))

        is_end = target_idx > len(self.path.cx) * 0.95

        return ControlMessage(0, 0, 2, 5, di, 0, 0), is_end

    def calc_angle(self, first_vec, second_vec):

        x1, y1 = first_vec[0], first_vec[1]
        x2, y2 = second_vec[0], second_vec[1]

        scale_first = np.hypot(x1, y1)
        scale_second = np.hypot(x2, y2)

        cos = (x1*x2 + y1*y2) / (scale_first * scale_second)
        theta = m.acos(cos)

        return theta

    def scan_stop_point(self):
        x, y = self.startPoint.x, self.startPoint.y
        _list = []  # list_of_CenterPoint

        path = rospkg.RosPack().get_path("parking") + "/parking/" + \
            rospy.get_param("/create_parking_area/parking_file", "parking2.csv")

        with open(path, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")

            for row in reader:

                row = map(float, row)
                _list.append([row[0], row[1]])
                quat1, quat2, quat3, quat4 = row[2], row[3], row[4], row[5]

            _, _, yaw = euler_from_quaternion([quat1, quat2, quat3, quat4])
        print(_list)
        Idx_stop_area = 1
        alpha_vec = [x-_list[Idx_stop_area][0], y-_list[Idx_stop_area][1]]
        beta_vec = [_list[0][0] - _list[1][0], _list[0][1] - _list[1][1]]

        beta = self.calc_angle([1, 0], beta_vec) - m.radians(90)
        alpha = self.calc_angle([1, 0], alpha_vec)

        scale_alpha_vec = np.hypot(alpha_vec[0], alpha_vec[1])


        len = scale_alpha_vec * m.cos(alpha - beta) / m.cos(yaw - beta)
        WP2_x, WP2_y = _list[Idx_stop_area][0] + len * \
            m.cos(yaw), _list[Idx_stop_area][1] + len * m.sin(yaw)

        return WP2_x, WP2_y

    def path_callback(self, msg):
        self.local_path = msg

    def main(self):
        is_end = False
        cmd = ControlMessage()

        parking_sequence_msg = 0
        parking_sequence_pub.publish(parking_sequence_msg)

        if self.parking_state.Searching:

            if self.startPoint is None:
                self.startPoint = Point(self.state.x, self.state.y, 0.)

            WP2_x, WP2_y = self.scan_stop_point()

            self.path = self.createPath(
                Point(WP2_x, WP2_y, 0.)) 

            if is_end != True:
                cmd, is_end = self.makeControlMessage(self.path)

            else:
                self.parking_state = ParkingState.Deceleration1
                parking_sequence_pub(self.parking_state)

        elif self.parking_state == ParkingState.Deceleration1:

            if self.state.v != 0:
                cmd = ControlMessage(0, 0, 1, 0, 0, self.brake, 0)
            else:
                self.parking_state = ParkingState.Reset
                parking_sequence_pub(self.parking_state)

        elif self.parking_state.Reset:
            self.path = self.createPath(self.startPoint)

            if is_end != True:
                cmd, is_end = self.makeControlMessage(self.path)
            else:
                self.parking_state = ParkingState.Deceleration2
                parking_sequence_pub(self.parking_state)

        elif self.parking_state == ParkingState.Deceleration2:

            if self.state.v != 0:
                cmd = ControlMessage(0, 0, 1, 0, 0, self.brake, 0)
            else:
                self.parking_state = ParkingState.Parking
                parking_sequence_pub(self.parking_state)

            # Call Function for Local Path Planning

        elif self.parking_state.Parking:

            rospy.wait_for_message('path', PathResponse)
            cmd, is_end = self.makeControlMessage(self.local_path)

            if is_end == True:
                self.parking_state = ParkingState.Deceleration3
                parking_sequence_pub(self.parking_state)

        elif self.parking_state == ParkingState.Deceleration3:

            if self.state.v != 0:
                cmd = ControlMessage(0, 0, 1, 0, 0, self.brake, 0)  
            else:
                self.parking_state = ParkingState.Backward
                parking_sequence_pub(self.parking_state)
                self.startPoint = Point(self.state.x, self.state.y, 0.0)

        elif self.parking_state.Backward:
            distance = np.hypot(
                self.state.x - self.startPoint.x,
                self.state.y - self.startPoint.y
            )

            if distance > 10:
                self.parking_state = ParkingState.End
                parking_sequence_pub(self.parking_state)

            else:
                cmd = ControlMessage(0, 0, 2, 5, 0, 0, 0)

        elif self.parking_state.End:
            pass

        else:
            rospy.logfatal("Invalid Parking State")

        return cmd


if __name__ == "__main__":
    rospy.init_node("test")

    state = OdomState("/odometry/kalman")
    stanley = Stanley()
    parking = VerticalParkingBase(state, stanley)

    rospy.Subscriber(
        "/path", PathResponse, callback=parking.path_callback)
    parking_sequence_pub = rospy.Publisher(
        '/parking_sequence', Int32, queue_size=3)
    
    while True:
        parking.main()
        pass
