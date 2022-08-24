#!/usr/bin/env python
# -*- coding: utf-8 -*-


from time import sleep
import rospy
import rospkg
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import *
from cubic_spline_planner import calc_spline_course
# from dynamic_window_approach import *
from parking_area import ParkingArea
from rrt_star_reeds_shepp import *
from std_msgs.msg import Int16, Float32MultiArray
from parking.msg import surround


def erase_other_zone():
    obstacle_list = []
    # left_big_circle
    left_big_radius = scale_y / m.tan(m.pi/4 - yaw/2)
    delta_x1 = -scale_y/2 - scale_x * m.tan(yaw)*m.sin(yaw)
    delta_y1 = -left_big_radius + m.cos(yaw)
    obstacle_list.append([points_of_parking_areas[target_area_Idx][2][0]+delta_x1,
                          points_of_parking_areas[target_area_Idx][2][1]+delta_y1, left_big_radius])

    # right_big_circle(upper)
    right_big1_radius = scale_x * m.tan(yaw)*m.tan(m.pi/4 - yaw/2)
    delta_x2 = scale_x * m.tan(yaw)
    delta_y2 = -right_big1_radius
    obstacle_list.append([points_of_parking_areas[target_area_Idx][3][0]+delta_x2,
                         points_of_parking_areas[target_area_Idx][3][1]+delta_y2, right_big1_radius])

    # right_big_circle(lower)
    right_big2_radius = scale_y - scale_x * \
        m.tan(yaw) * (1 + m.tan(m.pi/4 - yaw/2)) + 1
    delta_x4 = right_big2_radius * m.cos(yaw)
    delta_y4 = right_big2_radius * m.sin(yaw)
    obstacle_list.append([points_of_parking_areas[target_area_Idx][1][0]+delta_x2,
                         points_of_parking_areas[target_area_Idx][1][1]+delta_y2, right_big1_radius])

    # back small : 2
    # back_small_left
    back_small_radius = scale_x / 4
    delta_x_3 = (scale_x * m.cos(yaw)) / 4
    delta_y_3 = (scale_x * m.sin(yaw)) / 4
    obstacle_list.append([points_of_parking_areas[target_area_Idx][4][0] + delta_x_3,
                         points_of_parking_areas[target_area_Idx][4][1] + delta_y_3, back_small_radius])

    # back_small_right
    obstacle_list.append([points_of_parking_areas[target_area_Idx][4][0] + delta_x_3 * 3,
                         points_of_parking_areas[target_area_Idx][4][1] + delta_y_3 * 3, back_small_radius])

    # left_small_back
    obstacle_list.append([points_of_parking_areas[target_area_Idx][4][0] - delta_x_3 * 2,
                         points_of_parking_areas[target_area_Idx][4][1] - delta_y_3 * 2, scale_x/2])

    # obstacle_array = np.array(obstacle_list)

    return obstacle_list  # obstacle_array


def parking_zone_callback(msg):
    global target_area_Idx
    target_area_Idx = msg.data + 1
    rospy.loginfo("Subscribe target area idx")


def markerCallback(msg):  # subscribe 해온 msg 내용을 가용한 형태로 변환
    global points_of_parking_areas
    global list_of_center_point_list
    global yaw
    global scale_y
    global scale_x

    points_of_parking_areas = []
    list_of_center_point_list = []

    for marker in msg.markers:

        point = marker.pose.position
        orientation = marker.pose.orientation
        scale = marker.scale
        scale_y = scale.y
        scale_x = scale.x
        Parking_area = ParkingArea(
            x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x)

        points_of_parking_areas.append(Parking_area.parseArray().tolist())

        center_point_list = [point.x, point.y]  # 주차 라인 중앙 점 좌표
        list_of_center_point_list.append(
            center_point_list)  # 주차 라인 중앙 점 좌표 list

    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    # print(list_0f_point_list)
    # print(parking_areas_interval)

    rospy.loginfo("Subscribe parkin area MarkerArray")


if __name__ == "__main__":
    rospy.init_node("surround_obstacle")

    marker_sub = rospy.Subscriber(
        "/parking_areas", MarkerArray, callback=markerCallback)

    parking_zone_sub = rospy.Subscriber(
        '/parking_zone', Int16, callback=parking_zone_callback)

    surround_obstacle_list_pub = rospy.Publisher(
        '/erase_oher_zone', Float32MultiArray, queue_size=1)

    sleep(1.0)

    obstacle_msg = Float32MultiArray()

    obstacle_msg.data = erase_other_zone()

    hz = 1.
    freq = 1 / hz
    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        surround_obstacle_list_pub.publish(obstacle_msg)
        print(obstacle_msg.data)
        r.sleep()
