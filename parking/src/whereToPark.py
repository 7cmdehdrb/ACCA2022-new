#!/usr/bin/env python


import rospy
import rospkg
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Empty, Header, ColorRGBA
from geometry_msgs.msg import *
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from path_plan.msg import PathResponse
from erp42_control.msg import ControlMessage
from load_parking_area import loadCSV
from parking_area import ParkingArea
from cubic_spline_planner import calc_spline_course
from reeds_shepp_path_planning import reeds_shepp_path_planning
from enum import Enum
from time import sleep


class Where_to_park():
    def __init__(self):
        self.obstacles = []
        self.obstacle_zone = []
        self.scale_x, self.scale_y = 0, 0
        self.All_of_parking_area = 0
        self.parking_areas = ParkingArea()
        '''self.parking_areas = loadCSV(
            "/home/acca/catkin_ws/src/ACCA2022-new/parking/parking_csv/hor_parking.csv")'''
        self.tf_sub = tf.TransformListener()
        self.obstacle_sub = rospy.Subscriber("/adaptive_clustering/poses", PoseArray,
                                             callback=self.obstacleCallback)

    def obstacleCallback(self, msg):
        self.obstacle_zone = []
        self.poses = msg
        if self.tf_sub.canTransform("map", "velodyne", rospy.Time(0)):
            for p in self.poses.poses:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time(0)

                pose.pose.position.x = p.position.x
                pose.pose.position.y = p.position.y
                pose.pose.position.z = 0

                temp = self.tf_sub.transformPose(ps=pose, target_frame="map")
                self.obstacles.append(
                    [temp.pose.position.x, temp.pose.position.y])

    def checkIsInParking(obstacle, box):
        dist = np.hypot(box.position.x - obstacle[0],
                        box.position.y - obstacle[1])
        box_yaw = euler_from_quaternion(
            [box.orientation.x, box.orientation.y, box.orientation.z, box.orientation.w])

        if dist == 0.0:
            return True

        area_VEC = np.array([
            m.cos(box_yaw - m.radians(90.0)), m.sin(box_yaw - m.radians(90.0))
        ])

        ob_VEC = np.array([
            obstacle.x - box.x, obstacle.y - box.y
        ])

        theta = m.acos(np.dot(area_VEC, ob_VEC) / dist)

        x_dist = abs(dist * m.sin(theta))
        y_dist = abs(dist * m.cos(theta))

        if x_dist <= box.scale.x / 2.0 and y_dist <= box.scale.y / 2.0:
            return True

        return False

    def interval_checkIsInParking(self, state):  # state 넣는 방법..
        dist_list = []
        parking_areas = ParkingArea()
        state_x = state.x
        state_y = state.y
        # idx 찾기
        for _ in range(len(self.parking_areas)):
            dist = np.hypot(state_x - self.parking_areas.position.x,
                            state_y - self.parking_areas.position.y)
            dist_list.append(dist)

        Idx = np.argmin(np.array(dist_list))

        # 해당 idx의 parking area yaw 벡터와 내적하여 구간 내에 위치하는지 확인
        p_orientation = parking_areas[Idx].orientation
        pyaw = euler_from_quaternion(
            [p_orientation.x, p_orientation.y, p_orientation.z, p_orientation.w])

        pyaw_VEC = [m.cos(pyaw), m.sin(pyaw)]
        car_VEC = [state_x - parking_areas[Idx].x,
                   state_y - parking_areas[Idx].y]
        inner_product = abs(np.dot(pyaw_VEC, car_VEC))
        if inner_product < parking_areas.scale.y / 2:
            # 해당 구간 내에 있으면, 해당 idx의 parking area에 대하여 adaptive clustering data 계산하기
            for obstacle in self.obstacles:
                _bool = self.checkIsInParking(obstacle, parking_areas[Idx])
                if _bool == True:
                    self.obstacles = []
                    result_bool = True
                    break
                    # return True, Idx
        self.obstacles = []

        return result_bool, Idx


if __name__ == '__main__':
    rospy.init_node('where_to_park')

    WTP = Where_to_park()
