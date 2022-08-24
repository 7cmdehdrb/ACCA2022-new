#!/usr/bin/env python
# -*- coding: utf-8 -*-


from time import sleep
import math as m
from matplotlib.style import available
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int16
from parking_area import ParkingArea
import numpy as np
from copy import deepcopy
from tf.transformations import euler_from_quaternion

the_number_of_parkinarea = 6


def slope():
    x1 = parking_areas[0][1][0]
    x2 = parking_areas[1][1][1]
    y1 = parking_areas[0][1][0]
    y2 = parking_areas[1][1][1]

    slope = (y1-y2) / (x1 - x2)

    return slope

# 장애물의 y좌표와 같은 값을 y좌표로 가지는 주차 라인 위 점의 x좌표를 구하는 함수


def find_cool_x(upper_p, lower_p, y, scale_y, yaw):

    x = (upper_p[1]-y) / (scale_y * np.cos(yaw)) * \
        scale_y + min(upper_p[0], lower_p[0])

    return x


def linear_input_X_output_Y(o_x, o_y, x):

    y = slope() * (x - o_x) + o_y

    return y


def linear_input_Y_output_X(o_x, o_y, y):
    x = (y - o_y) / slope() + o_x

    return x


def interval_creator(y):  # 장애물의 y좌표와 같은 값을 y좌표로 가지는 주차 라인 위 점들의 x좌표 array

    interval = []
    interval.append(find_cool_x(
        parking_areas[0][4], parking_areas[0][2], y, scale_y, yaw))

    for i in parking_areas:
        interval.append(find_cool_x(i[3], i[1], y, scale_y, yaw))

    return interval


# 위 함수에서 구한 x interval 중 어느 사이에 장애물의 x 좌표가 위치하는지 알아낸다.
def where_is_obstacle(ob_x, ob_y):

    interval = interval_creator(ob_y)
    j = 1

    for i in range(len(interval)):
        if interval[i] < ob_x:
            j += 1
        else:
            break

    return j

# 1순위 : 3개이상 연속 또는 2개 연속이지만 끝 자리
# 2순위 : 2개 이상 연속 또는 연속자리는 아니지만 끝자리
# 3순위 : 1순위, 2순위 모두에 해당하지 않는 자리


available_zone = []
the_number_of_parkinarea = 6


def where_to_park(available_zone):
    available_zone.insert(0, 0)
    available_zone.append(the_number_of_parkinarea)
    row = []
    area_in_a_row = []
    temp_of_area_in_a_row = []
    j = -3
    k = 0
    number_of_in_a_row = 0

    for i in available_zone:
        if i != j + 1:
            row.append(number_of_in_a_row)
            area_in_a_row.append(temp_of_area_in_a_row)
            temp_number_of_in_a_row = 0
            temp_of_area_in_a_row = []
            if len(area_in_a_row) < len(row):
                area_in_a_row.append([])
                k += 1
        else:
            temp_number_of_in_a_row += 1
            temp_of_area_in_a_row.append(i)
    if max(row) == 0:
        result = available_zone[1]
    else:
        Max = max(row)
        loc_of_Max = row.index(Max)
        candidate_area = area_in_a_row[loc_of_Max]
        result = candidate_area[int(m.ceil(len(candidate_area)/2))]

    return result


def markerCallback(msg):  # subscribe 해온 msg 내용을 가용한 형태로 변환
    global parking_areas
    global point_list
    global yaw
    global scale_y

    list_of_point_list = []

    for marker in msg.markers:

        point = marker.pose.position
        orientation = marker.pose.orientation
        scale = marker.scale
        scale_y = scale.y
        Parking_area = ParkingArea(
            x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x)

        parking_areas.append(Parking_area.parseArray().tolist())

        point_list = [point.x, point.y]  # 주차 라인 중앙 점 좌표
        list_of_point_list.append(point_list)  # 주차 라인 중앙 점 좌표 list

    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    # print(list_0f_point_list)
    # print(parking_areas_interval)

    rospy.loginfo("Subscribe parkin area MarkerArray")


def obstacleCallback(msg):

    global list_of_obstalce_x  # 장애물 좌표들의 리스트
    global list_of_obstalce_y

    list_of_obstalce_x = []
    list_of_obstalce_y = []

    for marker in msg.markers:
        point = marker.pose.position

        list_of_obstalce_x.append(point.x)
        list_of_obstalce_y.append(point.y)

    rospy.loginfo("Subscribe obstacle MarkerArray")


if __name__ == "__main__":
    rospy.init_node("decide_whereToPark")

    # state = State("/odometry/kalman")
    parking_areas = []
    available_zone = []

    marker_sub = rospy.Subscriber(
        "/parking_areas", MarkerArray, callback=markerCallback)

    obstacle_sub = rospy.Subscriber(
        "/obstacle_in_parking_areas", MarkerArray, callback=obstacleCallback)

    sleep(3.0)
    for x, y in zip(list_of_obstalce_x, list_of_obstalce_y):

        k = where_is_obstacle(x, y)

        available_zone.append(k)

    # parking_zone = where_to_park(available_zone)
    target_zone_msg = Int16()
    target_zone_msg.data = where_to_park(available_zone)

    parking_zone_pub = rospy.Publisher('/parking_zone', Int16, queue_size=1)

    hz = 1.
    freq = 1 / hz
    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        parking_zone_pub.publish()
        print(where_to_park(available_zone))

        r.sleep()
