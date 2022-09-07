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
# from parking.msg import surround_obstacle


import rospy
from geometry_msgs.msg import PointStamped
# import tf
# import random
import numpy as np

global x, y, z, waypoints
x = 0.0
y = 0.0
z = 0.0
points = []
waypoints = None
scale_x, scale_y = 2.5, 5.0
# parkinglot start, parkinglot finish
# waypoint = {'parkinglot_start' : [0,0], 'parking_finish' : [0,0], 'parking_finish' : 0, 'parking_yf' : 0, }


def calc_yaw(a, b):

    dist = np.hypot(a, b)

    cos = b / dist

    theta = m.acos(cos)

    return theta


def callback(msg):
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "/map"
    point.point.x = x
    point.point.y = y
    # point.point.z = z
    temp = [x, y]
    if len(points) <= 9:
        points.append(temp)

    else:
        waypoints = np.array(temp)


def parking_lot_creator(points):
    if len(points) < 9:
        rospy.loginfo('You need to add more point')
        sleep(3.0)
        parking_lot_creator(points)
    else:
        result = []

        # center point of first parking lot
        temp_xs, temp_ys = 0, 0
        for i in range(4):
            temp_xs += points[i][0]
            temp_ys += points[i][1]

        # scale_x, scale_y

        scale_x_x = (abs(points[0][0] - points[1][0]) +
                     abs(points[2][0] - points[3][0])) / 2
        scale_x_y = (abs(points[0][1] - points[1][1]) +
                     abs(points[2][1] - points[3][1])) / 2
        scale_x = np.hypot(scale_x_x, scale_x_y)

        scale_y_x = (abs(points[0][0] - points[3][0]) +
                     abs(points[1][0] - points[2][0])) / 2
        scale_y_y = (abs(points[0][1] - points[3][1]) +
                     abs(points[1][1] - points[2][1])) / 2
        scale_y = np.hypot(scale_y_x, scale_y_y)

        # interval
        temp_interval_x, temp_interval_y = 0, 0
        for j in range(5):
            k = j+4
            temp_interval_x += points[k][0] - points[k-1][0]
            temp_interval_y += points[k][1] - points[k-1][1]

        xs, ys = temp_xs / 4, temp_ys / 4
        interval_x, interval_y = temp_interval_x / \
            4, temp_interval_y / 4  # 4번째 칸의 좌측 전방 꼭짓점

        first_lot = [xs, ys]
        interval = [interval_x, interval_y]

        # yaw

        cos = interval_x / (interval_x**2 + interval_y**2)
        yaw = m.acos(cos)

        quat = quaternion_from_euler(0.0, 0.0, yaw, 'rxyz')

        for i in range(6):

            x = [first_lot[0] + interval_x[0] * i, first_lot[1] + interval[0] *
                 i, quat[1][0], quat[1][1], quat[1][2], quat[1][3], scale_x, scale_y]

            result.append(x)

        return result


def scan_stop_point(stop_parkinglot):

    _, _, alpha = euler_from_quaternion(
        [stop_parkinglot[2], stop_parkinglot[3], stop_parkinglot[4], stop_parkinglot[5]])

    x = stop_parkinglot[0] + m.cos(alpha) * 1

    y = stop_parkinglot[1] + m.sin(alpha)

    return x, y


def save_waypoints_csv(parking_areas):

    path = rospkg.RosPack().get_path("parking") + "/parking/" + \
        rospy.get_param("/create_parking_area/parking_file", "waypoints.csv")

    x, y = scan_stop_point(parking_areas[3])

    with open(path, 'w') as csvfile:
        csvfile.write(str(waypoints[0][0]) + ',' + str(waypoints[0][1]) + ',' + '\n'
                      + str(x) + ',' + str(y) + '\n'
                      + str(waypoints[0][0]) + ',' + str(waypoints[0][1]) + ',' + '\n')

    rospy.loginfo("Save Way Points! : %s" % path)


def save_parking_area_csv(parking_areas):

    path = rospkg.RosPack().get_path("parking") + "/parking/" + \
        rospy.get_param("/create_parking_area/parking_file", "my_parking.csv")

    with open(path, 'w') as csvfile:
        for i in range(len(parking_areas)):

            # x, y, ox, oy, oz, ow, h, w
            csvfile.write(
                str(parking_areas[i][0]) + "," +
                str(parking_areas[i][1]) + "," +
                str(parking_areas[i][2]) + "," +
                str(parking_areas[i][3]) + "," +
                str(parking_areas[i][4]) + "," +
                str(parking_areas[i][5]) + "," +
                str(parking_areas[i][6]) + "," +
                str(parking_areas[i][7]) + "\n"
            )

        rospy.loginfo("Save Parking Area! : %s" % path)


if __name__ == '__main__':

    rospy.init_node('goal_publisher', anonymous=True)
    point_sub = rospy.Subscriber(
        '/clicked_point', PointStamped, callback)

    parking_areas = parking_lot_creator(points)
    save_parking_area_csv(parking_areas)
    scan_stop_point(parking_areas[3])
    save_waypoints_csv(parking_areas)
