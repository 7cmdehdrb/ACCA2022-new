#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
from turtle import pos
import rospy
import rospkg
import numpy as np
import math as m
from nav_msgs.msg import Path
from geometry_msgs.msg import *
from path_plan.msg import PathResponse
from erp42_control.msg import ControlMessage
from cubic_spline_planner import calc_spline_course
from abc import *
from enum import Enum
from std_msgs.msg import Int8
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
from nav_msgs.msg import Odometry
from target_selector_modify import TargetSelector
from local_path_planner import Local_path_planner
from state import State, OdomState
from stanley import Stanley

'''try:
    sys.path.append(rospkg.RosPack().get_path("erp42_control") + "/src")
    from stanley import Stanley
    from state import State, OdomState
except Exception as ex:
    rospy.logfatal("Import Error : Vertical Parking")
    rospy.logfatal(ex)'''


class ParkingState(Enum):
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

    def __init__(self, state, stanley):
        self.state = state
        self.stanley = stanley
        self.path = PathResponse()
        self.local_path = PathResponse()

        self.startPoint = None
        self.parking_state = ParkingState.Searching
        self.gear = 1
        self.target_idx = 0
        self.brake = 70
        self.trig = True
        self.is_end = False
        self.trigger = False
        self.parking_state_msg = Int8()
        self.parking_state_msg.data = 0
        self._list = []  # list_of_CenterPoint

    def path_callback(self, msg):
        self.local_path = msg

    def toRosPath(self, xs, ys, yaws):
        # ros path publish
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for i in range(len(xs)):

            pose = PoseStamped()

            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = xs[i]
            pose.pose.position.y = ys[i]

            quat = quaternion_from_euler(0., 0., yaws[i])

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            path.poses.append(pose)

        rospath_pub.publish(path)

    def createPath(self, end_point=Point()):
        cx, cy, cyaw, _, _ = calc_spline_course([self.state.x, end_point.x], [
            self.state.y, end_point.y], ds=0.1)

        self.toRosPath(cx, cy, cyaw)

        self.path = PathResponse()
        self.path.cx = cx
        self.path.cy = cy
        self.path.cyaw = cyaw

        return self.path

    def makeControlMessage(self, path):
        di, target_idx = self.stanley.stanley_control(
            state=self.state,
            cx=path.cx,
            cy=path.cy,
            cyaw=path.cyaw,
            last_target_idx=self.target_idx
        )
        # reverse 값 추가

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(30.0), m.radians(30.0))

        self.is_end = target_idx > len(self.path.cx) * 0.90
        print('target_idx : %f, is_end : %s' % (self.target_idx, self.is_end))

        return ControlMessage(0, 0, self.gear, 5, di, 0, 0), self.is_end

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

        path = rospkg.RosPack().get_path("parking") + "/parking/" + \
            rospy.get_param(
                "/create_parking_area/parking_file", "parking3.csv")

        with open(path, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")

            for row in reader:

                row = map(float, row)
                self._list.append([row[0], row[1]])
                quat1, quat2, quat3, quat4 = row[2], row[3], row[4], row[5]

            _, _, self.pyaw = euler_from_quaternion(
                [quat1, quat2, quat3, quat4])

        Idx_stop_area = len(self._list) - 3

        # parking 시작 위치부터 Idx_stop_area 중심점까지 벡터
        alpha_VEC = [self._list[Idx_stop_area][0] -
                     x, self._list[Idx_stop_area][1] - y]
        beta_VEC = [self._list[1][0] - self._list[0][0], self._list[1]
                    [1] - self._list[0][1]]  # 주차장 결대로 그은 벡터
        gamma_VEC = [m.cos(self.pyaw), m.sin(self.pyaw)]  # 주차장 정의하는 방향 벡터

        angle_a = self.calc_angle(alpha_VEC, beta_VEC)
        angle_b = self.calc_angle(alpha_VEC, gamma_VEC)

        if angle_a >= m.pi/2:
            angle_a = m.pi - angle_a

        if angle_b >= m.pi/2:
            angle_b = m.pi - angle_b
            sign = 1
        else:
            sign = -1

        angle_c = m.pi - angle_a - angle_b
        scale_alphaVEC = np.hypot(alpha_VEC[0], alpha_VEC[1])
        self.scale_YawVEC = scale_alphaVEC * (m.sin(angle_a)/m.sin(angle_c))

        WP2_x = self._list[Idx_stop_area][0] + \
            self.scale_YawVEC * m.cos(self.pyaw) * sign
        WP2_y = self._list[Idx_stop_area][1] + \
            self.scale_YawVEC * m.sin(self.pyaw) * sign

        ###################
        self.standard_x, self.standard_y = self._list[Idx_stop_area][0], self._list[Idx_stop_area][1]

        rospy.loginfo('length of vecter: %f' % self.scale_YawVEC)
        print(self._list)
        print(WP2_x, 'WP2_x')
        print(WP2_y, 'WP2_y')
        print('x:', x)
        print('y:', y)
        return WP2_x, WP2_y

    def WP3_creator(self, target_Zone):
        reverse_beta_VEC = [-self._list[1][0] + self._list[0][0], -self._list[1]
                            [1] + self._list[0][1]]  # 주차장 결대로 그은 벡터의 역방향

        scale_of_beta_VEC = np.hypot(reverse_beta_VEC[0], reverse_beta_VEC[1])

        beta_yaw = [reverse_beta_VEC[0] / scale_of_beta_VEC,
                    reverse_beta_VEC[1] / scale_of_beta_VEC]

        target_Zone_idx = target_Zone - 1

        self.target_cool_x, self.target_cool_y = self._list[
            target_Zone_idx][0], self._list[target_Zone_idx][1]

        WP3_x, WP3_y = self.target_cool_x - self.scale_YawVEC * m.cos(self.pyaw) + 4 * \
            beta_yaw[0], self.target_cool_y - self.scale_YawVEC * \
            m.sin(self.pyaw) + 4 * beta_yaw[1]

        return WP3_x, WP3_y

    def point_Pub(self, x, y):

        point_stamped = PointStamped()
        point = Point()

        point_stamped.header.frame_id = "map"
        point_stamped.header.stamp = rospy.Time.now()

        point.x = x
        point.y = y
        point.z = 0

        point_stamped.point = point

        point_pub.publish(point_stamped)

    def current_loc_pub(self, x, y):
        point_stamped = PointStamped()
        point = Point()

        point_stamped.header.frame_id = "map"
        point_stamped.header.stamp = rospy.Time(0)

        point.x = x
        point.y = y
        point.z = 0

        point_stamped.point = point

        current_loc_pub.publish(point_stamped)

    #################
    def standard_pub(self, x, y):

        point_stamped = PointStamped()
        point = Point()

        point_stamped.header.frame_id = "map"
        point_stamped.header.stamp = rospy.Time(0)

        point.x = x
        point.y = y
        point.z = 0

        point_stamped.point = point

        standard_pub.publish(point_stamped)

    def startpose_pub(self, x, y):
        pose = Pose()
        point = Point()
        quat = Quaternion()

        point.x = x
        point.y = y
        point.z = 0
        pose.position = point

        quat = quaternion_from_euler(0., 0., self.start_yaw)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        start_pose_pub.publish(pose)

    def main(self):
        cmd = ControlMessage()

        rospy.loginfo(str(self.parking_state))
        rospy.wait_for_message('/odometry/kalman', Odometry)
        if self.parking_state == ParkingState.Searching:
            self.gear = 2
            if self.startPoint is None:
                rospy.loginfo(str(state.x))
                rospy.loginfo(str(state.y))
                self.startPoint = Point(self.state.x, self.state.y, 0.)
                self.start_yaw = state.yaw
                # for inside test
                #####################################################################################
                # self.WP2_x, self.WP2_y = self.scan_stop_point()
                # self.scan_stop_point(self.startPoint)
                # self.WP2_x, self.WP2_y = 15.188313,  10.277684

                self.path = self.createPath(
                    Point(15.188313,  10.277684, 0))
                self.WP2_x, self.WP2_y = self.scan_stop_point()

                # for outside test

                '''self.WP2_x, self.WP2_y = self.scan_stop_point()

                self.path = self.createPath(
                    Point(self.WP2_x, self.WP2_y, 0.))'''

            self.current_loc_pub(self.state.x, self.state.y)

            self.toRosPath(self.path.cx, self.path.cy, self.path.cyaw)
            target_selector.checkIsInParking()

            if self.is_end == False:
                cmd, self.is_end = self.makeControlMessage(self.path)

            else:
                self.parking_state = ParkingState.Deceleration1
                self.parking_state_msg.data += 1
                self.is_end = False
                self.target_idx = 0

        elif self.parking_state == ParkingState.Deceleration1:
            ######################################################################
            if self.state.v <= 3:
                cmd = ControlMessage(0, 0, 2, 0, 0, self.brake, 0)

            else:
                self.parking_state = ParkingState.Reset
                self.parking_state_msg.data += 1

        elif self.parking_state == self.parking_state.Reset:

            if self.trigger == False:
                self.gear = 0
                self.path = self.createPath(self.startPoint)
                self.trigger = True
                self.target_zone_number = target_selector.where_to_park(
                    target_selector.All_of_parking_area)

                self.WP3_x, self.WP3_y = self.WP3_creator(
                    target_Zone=self.target_zone_number)
                self.WP3_Point = Point(
                    float(self.WP3_x), float(self.WP3_y), 0.)
                self.target_zone_msg = Int8()
                self.target_zone_msg.data = self.target_zone_number
            WP3_pub.publish(self.WP3_Point)
            self.point_Pub(self.WP3_x, self.WP3_y)
            target_zone_pub.publish(self.target_zone_msg)
            self.toRosPath(self.path.cx, self.path.cy, self.path.cyaw)
            if self.is_end == False:
                '------------------------------------------------------------------------------'
                try:
                    cmd, self.is_end = self.makeControlMessage(self.path)
                except:
                    print('error in reset state -> makeControlMessage')
                '------------------------------------------------------------------------------'
                print('path_len', len(self.path.cx))
            else:
                self.parking_state = ParkingState.Deceleration2
                self.parking_state_msg.data += 1
                self.is_end = False
                self.target_idx = 0

        elif self.parking_state == ParkingState.Deceleration2:
            if self.state.v <= 3:
                cmd = ControlMessage(0, 0, 0, 0, 0, self.brake, 0)
            else:
                self.parking_state = ParkingState.Parking
                self.parking_state_msg.data += 1

            # Call Function for Local Path Planning

        elif self.parking_state == self.parking_state.Parking:
            self.gear = 2
            rospy.wait_for_message('/path', PathResponse)
            try:
                cmd, self.is_end = self.makeControlMessage(self.local_path)
            except:
                pass

            if self.is_end == True:
                self.parking_state = ParkingState.Deceleration3
                self.parking_state_msg.data += 1
                self.is_end = False
                self.target_idx = 0

        elif self.parking_state == ParkingState.Deceleration3:
            if self.state.v != 0:
                cmd = ControlMessage(0, 0, 2, 0, 0, self.brake, 0)
            else:
                self.parking_state = ParkingState.Backward
                self.parking_state_msg.data += 1
                self.target_zone = Point(self.state.x, self.state.y, 0.0)

        elif self.parking_state == self.parking_state.Backward:
            distance = np.hypot(
                self.state.x - self.target_zone.x,
                self.state.y - self.target_zone.y
            )

            if distance > 3:
                self.parking_state = ParkingState.End
                self.parking_state_msg.data += 1

            else:
                cmd = ControlMessage(0, 0, 0, 5, 0, 0, 0)

        elif self.parking_state == self.parking_state.End:
            pass

        else:
            rospy.logfatal("Invalid Parking State")

        parking_state_pub.publish(self.parking_state_msg)
        self.startpose_pub(self.startPoint.x, self.startPoint.y)
        self.current_loc_pub(self.state.x, self.state.y)
        cmd_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("parking_maintest")

    target_selector = TargetSelector()
    local_path_planner = Local_path_planner()
    state = OdomState("/odometry/kalman")
    stanley = Stanley()
    parking = VerticalParkingBase(state, stanley)
    parking_state_pub = rospy.Publisher(
        '/parking_sequence', Int8, queue_size=3)

    path_sub = rospy.Subscriber(
        "/path", PathResponse, callback=parking.path_callback)

    rospath_pub = rospy.Publisher("/searching_path", Path, queue_size=1)

    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=2)

    point_pub = rospy.Publisher("/point_pub", PointStamped, queue_size=1)

    marker_sub = rospy.Subscriber(
        "/parking_areas", MarkerArray, callback=target_selector.markerCallback)

    obstacle_sub = rospy.Subscriber(
        "/adaptive_clustering/poses", PoseArray, callback=target_selector.obstacleCallback)

    target_zone_pub = rospy.Publisher('/target_zone', Int8, queue_size=1)

    current_loc_pub = rospy.Publisher(
        "/current_location", PointStamped, queue_size=1)

    start_pose_pub = rospy.Publisher(
        "/startpose", Pose, queue_size=1)

    WP3_pub = rospy.Publisher('/parking_WP3',  Point, queue_size=1)

    standard_pub = rospy.Publisher(
        "/standard", PointStamped, queue_size=1)

    r = rospy.Rate(20.)
    while not rospy.is_shutdown():
        if parking.trig == True:
            parking.main()

        r.sleep()
