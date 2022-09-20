#!/usr/bin/env python
# -*- coding: utf-8 -*-

from distutils.archive_util import make_archive
from time import sleep
import math as m
import rospy
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Int8
from parking_area import ParkingArea
from std_msgs.msg import ColorRGBA
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from load_parking_area import loadCSV
import rospkg


class TargetSelector():
    def __init__(self):
        self.markers = MarkerArray()
        self.poses = PoseArray()
        self.center_points = []
        self.parking_areas = []
        self.obstacles = []
        self.pyaw = []
        self.scale_y = 0
        self.scale_x = 0
        self.obstacle_zone = []
        self.go_next = False
        self.tf_sub = tf.TransformListener()
        self.parking_state = 0
        self.All_of_parking_area = [1, 2, 3, 4, 5, 6]
        self.the_number_of_parkingarea = 6
        self.map_obstacle_pub = rospy.Publisher(
            "/map_obstacle", PoseArray, queue_size=1)
        self.csv_path = rospkg.RosPack().get_path("parking") + "/parking/" + \
            rospy.get_param("/create_parking_area/parking_file",
                            "parking3.csv")
        self.base = None

    def markerCallback(self, msg):
        self.markers = msg

        for marker in self.markers.markers:
            point = marker.pose.position
            orientation = marker.pose.orientation
            scale = marker.scale
            self.scale_y = scale.y
            self.scale_x = scale.x
            parking_area = ParkingArea(
                x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x)

            self.parking_areas.append(parking_area)

            _, _, yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])

            center_point = [point.x, point.y, yaw]  # 주차 라인 중앙 점 좌표
            self.center_points.append(center_point)  # 주차 라인 중앙 점 좌표 list

        rospy.loginfo("Subscribe parking area MarkerArray")

        marker_sub.unregister()

    # FOR random_obstacle
    # def obstacleCallback(self, msg):
    #     self.obstacle_markers = msg

    #     for marker in self.obstacle_markers:
    #         point = marker.pose.position
    #         self.obstacles.append([point.x, point.y])

    #     rospy.loginfo("Subscribe obstacle MarkerArray")

    # FOR adaptive_clustering

    def obstacleCallback(self, msg):
        self.obstacle_zone = []
        self.poses = msg
        posearray = PoseArray()
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

                posearray.poses.append(temp.pose)

            posearray.header.frame_id = "map"
            posearray.header.stamp = rospy.Time(0)

            self.map_obstacle_pub.publish(posearray)

        else:
            rospy.logwarn(
                "Cannot lookup transform between map and base_link : target_selector_modify.py")

    def SequenceCallback(self, msg):
        self.parking_state = msg.data

        #print(self.parking_state < 2)

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

    def inerval_checkIsInParking(self, state):
        dist_list = []
        state_x = state.x
        state_y = state.y
        for _ in range(len(self.parking_areas)):
            dist = np.hypot(state_x - self.parking_areas.position.x,
                            state_y - self.parking_areas.position.y)
            dist_list.append(dist)

        Idx = np.argmin(np.array(dist_list))

        # 해당 idx의 parking area yaw 벡터와 내적하여 구간 내에 위치하는지 확인
        p_orientation = self.parking_areas[Idx].orientation
        pyaw = m.pi/2 + euler_from_quaternion(
            [p_orientation.x, p_orientation.y, p_orientation.z, p_orientation.w])

        pyaw_VEC = [m.cos(pyaw), m.sin(pyaw)]
        car_VEC = [state_x - self.parking_areas[Idx].x,
                   state_y - self.parking_areas[Idx].y]

        inner_product = np.abs(np.dot(pyaw_VEC, car_VEC))
        if inner_product < self.parking_areas[Idx].scale.x / 2:
            # 해당 구간 내에 있으면, 해당 idx의 parking area에 대하여 adaptive clustering data 계산하기
            for obstacle in self.obstacles:
                _bool = self.checkIsInParking(
                    obstacle, self.parking_areas[Idx])
                if _bool == True:
                    self.obstacles = []
                    result_bool = True
                    break
                    # return True, Idx
        self.obstacles = []

        return result_bool, Idx

    def available_area(self):
        if self.base == None:
            self.base = []
            for _ in range(self.parking_areas):
                self.base.append(0)

    def where_to_park(self, available_zone):

        if len(available_zone) == 0:
            result = 4
        else:
            available_zone.insert(0, 0)
            available_zone.append(self.the_number_of_parkingarea+1)
            row = []  # save continuous times
            area_in_a_row = []  # save continuous parking area
            # (temporary list) save continuous parking area
            temp_of_area_in_a_row = [0]
            j = 0  # save old parking area temporary
            temp_number_of_in_a_row = 1  # save the number of continuous parking area

            for i in available_zone:
                if i == 0:
                    continue

                if i == j + 1:
                    if j not in temp_of_area_in_a_row:
                        temp_of_area_in_a_row.append(j)
                    temp_number_of_in_a_row += 1
                    temp_of_area_in_a_row.append(i)
                    j = i

                else:
                    if temp_number_of_in_a_row == [0]:
                        temp_of_area_in_a_row = []
                        continue
                    row.append(temp_number_of_in_a_row)
                    area_in_a_row.append(temp_of_area_in_a_row)
                    temp_number_of_in_a_row = 1
                    temp_of_area_in_a_row = []
                    j = i
            row.append(temp_number_of_in_a_row)
            area_in_a_row.append(temp_of_area_in_a_row)

            if max(row) == 1:
                result = available_zone[1]

            else:
                Max = max(row)
                loc_of_Max = row.index(Max)
                candidate_area = area_in_a_row[loc_of_Max]
                print('row', row)
                print('area_in_a_row', area_in_a_row)
                print('candidate', candidate_area)
                print('len', len(candidate_area)/2.0)
                print('index', int(m.ceil(len(candidate_area)/2.0))-1)
                result = candidate_area[int(m.ceil(len(candidate_area)/2.0))-1]
                if result == 0:
                    result = 1
                elif result == 7:
                    result = 6

        return result


if __name__ == "__main__":
    rospy.init_node("target_selector")

    target_select = TargetSelector()

    # for adaptive clustering

    target_zone_pub = rospy.Publisher('/target_zone', Int8, queue_size=1)

    map_obstacle_pub = rospy.Publisher(
        "/map_obstacle", PoseArray, queue_size=1)

    marker_sub = rospy.Subscriber(
        "/parking_areas", MarkerArray, callback=target_select.markerCallback)

    # FOR adaptive_clustering
    obstacle_sub = rospy.Subscriber(
        "/adaptive_clustering/poses", PoseArray, callback=target_select.obstacleCallback)

    parking_sequence_sub = rospy.Subscriber(
        '/parking_sequence', Int8, callback=target_select.SequenceCallback)

    hz = 1.
    freq = 1 / hz

    r = rospy.Rate(10.)
    while not rospy.is_shutdown():
        rospy.loginfo('parking state = %d' % target_select.parking_state)
        if target_select.parking_state < 2:
            target_select.checkIsInParking()
            rospy.loginfo('accepting obstacle')

        elif (target_select.parking_state >= 2) and (target_select.go_next == False):

            target_zone = target_select.where_to_park(
                target_select.All_of_parking_area)

            # parking_zone = where_to_park(available_zone)
            target_zone_msg = Int8()
            target_zone_msg.data = target_zone

            target_select.go_next = True

        else:
            pass
        while target_select.go_next == True:
            target_zone_pub.publish(target_zone_msg)
            print('result', target_zone)

        r.sleep()
