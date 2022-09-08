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

    def markerCallback(self, msg):
        self.markers = msg

        for marker in self.markers.markers:
            point = marker.pose.position
            orientation = marker.pose.orientation
            scale = marker.scale
            self.scale_y = scale.y
            self.scale_x = scale.x
            Parking_area = ParkingArea(
                x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x)

            self.parking_areas.append(Parking_area.parseArray().tolist())

            _, _, yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])

            center_point = [point.x, point.y, yaw]  # 주차 라인 중앙 점 좌표
            self.center_points.append(center_point)  # 주차 라인 중앙 점 좌표 list

        rospy.loginfo("Subscribe parking area MarkerArray")

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

    def checkIsInParking(self):
        for obstacle in self.obstacles:

            for i, point in enumerate(self.center_points):
                dist = np.hypot(
                    point[0] - obstacle[0], point[1] - obstacle[1])

                if dist == 0.0:
                    return True

                area_VEC = np.array([
                    m.cos(point[2] - m.radians(90.0)
                          ), m.sin(point[2] - m.radians(90.0))
                ])

                ob_VEC = np.array(
                    [(obstacle[0] - point[0]), (obstacle[1] - point[1])])

                abs_multiple = np.sqrt(
                    area_VEC[0]**2+area_VEC[1]**2) * np.sqrt(ob_VEC[0]**2+ob_VEC[1]**2)

                theta = m.acos(np.dot(area_VEC, ob_VEC) / abs_multiple)

                x_dist = abs(dist * m.sin(theta))
                y_dist = abs(dist * m.cos(theta))

                if x_dist <= self.scale_x / 2.0 and y_dist <= self.scale_y / 2.0:

                    if i+1 not in self.obstacle_zone:
                        self.obstacle_zone.append(i+1)

        for ob_zone in self.obstacle_zone:
            try:
                self.All_of_parking_area.remove(ob_zone)
            except:
                pass

    def where_to_park(self, available_zone):

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
