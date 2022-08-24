#!/usr/bin/env python
# -*- coding: utf-8 -*-


# from matplotlib import scale
import rospy
import rospkg
import numpy as np
import math as m
from time import sleep
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from genpy import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA
import random
from parking_area import ParkingArea
from nav_msgs.msg import Odometry

the_number_of_obstacle = 3
parking_area_w = 0.8


class Static_obstacle(object):
    def __init__(self, the_number_of_obstacle, parking_area_w, parking_areas, y=4.0):
        self.n = the_number_of_obstacle
        self.parking_areas = parking_areas
        self.ob_y = y
        self.parking_area_w = parking_area_w

    def random_obstacle(self):
        center_of_parking_area = []
        # 주차장 칸의 중간점 x 좌표 리스트
        for j in range(self.n):
            a = 4 + random.randrange(0, 6, 1) * 2.8
            while a in center_of_parking_area:
                a = 4 + random.randrange(0, 6, 1) * 2.8
            center_of_parking_area.append(a)
            center_of_parking_area.sort()

        print(center_of_parking_area)

        obstacle_list = []
        for o_x in center_of_parking_area:
            # 중간점에서 조금 벗어난 위치에 장애물 위치 지정
            x = o_x + (0.005 * random.randrange(
                int(-self.parking_area_w * 5), int(self.parking_area_w * 5), 2))  # *5 <== /2 and *10
            y = self.ob_y + 0.5*(random.random() *
                                 self.parking_area_w * m.sin(yaw))
            print(x, y)
            print(type(x), type(y))
            cool_of_ob = [x, y]
            print(cool_of_ob)
            obstacle_list.append(cool_of_ob)

        return obstacle_list


def markerCallback(msg):
    global parking_areas
    global point_list
    global yaw
    global scale_y
    global scale_x

    list_0f_point_list = []

    for marker in msg.markers:

        point = marker.pose.position
        orientation = marker.pose.orientation
        scale = marker.scale
        scale_x = scale.x
        scale_y = scale.y
        Parking_area = ParkingArea(
            x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x)

        parking_areas.append(list(Parking_area.parseArray()))

        point_list = [point.x, point.y]
        list_0f_point_list.append(point_list)
        # print(parking_areas)

    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])

    # print(parking_areas_interval)

    rospy.loginfo("Subscribe MarkerArray")

    return point_list


def parseMarker(id, position, duration=1):
    marker = Marker()

    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.4
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.3
    marker.scale.y = 0.3

    marker.color = ColorRGBA(0., 1., 0., 0.2)

    marker.type = 1
    marker.id = id
    marker.ns = str(id)
    marker.lifetime = Duration(secs=duration)

    return marker


if __name__ == "__main__":
    rospy.init_node("create_obstacle")
    parking_areas = []

    rospy.wait_for_message("/parking_areas", MarkerArray)
    pub = rospy.Publisher("/obstacle_around_parking_areas",
                          MarkerArray, queue_size=1)
    marker_sub = rospy.Subscriber(
        "/parking_areas", MarkerArray, callback=markerCallback)

    sleep(1.)

    hz = 1.
    freq = 1 / hz

    static_ob = Static_obstacle(
        the_number_of_obstacle, parking_area_w, parking_areas)

    ob_list = static_ob.random_obstacle()

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        msg = MarkerArray()

        for i, parking in enumerate(ob_list):
            msg.markers.append(parseMarker(
                id=i, position=ob_list[i], duration=int(freq)))

        pub.publish(msg)

        r.sleep()
