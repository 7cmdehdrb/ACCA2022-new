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
from parking_area import ParkingArea
from cubic_spline_planner import calc_spline_course
from time import sleep

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State, OdomState
except Exception as ex:
    rospy.logfatal(ex)


def markerCallback(msg):
    global parking_areas

    for marker in msg.markers:
        point = marker.pose.position
        orientation = marker.pose.orientation
        scale = marker.scale

        parking_areas.append(ParkingArea(
            x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x))

    rospy.loginfo("Subscribe MarkerArray")

    parking_sub.unregister()


def createPath(circle1, circle2, selected_parking_area):
    global path_pub

    _, _, yaw = euler_from_quaternion([
        selected_parking_area.orientation.x,
        selected_parking_area.orientation.y,
        selected_parking_area.orientation.z,
        selected_parking_area.orientation.w
    ])

    # std_vec : Unit Vector, cross_vec : Vector from circle1 point to contact point
    std_vec = np.array([m.cos(yaw), m.sin(yaw)])
    cross_vec = np.array(
        [circle1.pose.position.x - circle2.pose.position.x, circle1.pose.position.y - circle2.pose.position.y])

    # orientation gap between heading angle of parking lot and contact point
    theta1 = m.acos(np.dot(std_vec, cross_vec) /
                    ((np.hypot(std_vec[0], std_vec[1])) *
                    np.hypot(cross_vec[0], cross_vec[1])))

    # theta range for circle 1
    yaw_start = yaw + m.pi / 2
    yaw_end = yaw + theta1
    yaw_range = np.arange(yaw_start, yaw_end, 0.01 *
                          (1.0 if yaw_end > yaw_start else -1.0))

    # Set start point
    start_x = circle2.pose.position.x + \
        circle2.scale.x / 2.0 * \
        m.cos(yaw_range[0]) + circle2.scale.x / 2.0 * m.cos(yaw)
    start_y = circle2.pose.position.y + \
        circle2.scale.x / 2.0 * \
        m.sin(yaw_range[0]) + circle2.scale.x / 2.0 * m.sin(yaw)

    xs1 = [circle2.pose.position.x + circle2.scale.x /
           2.0 * m.cos(y) for y in yaw_range]
    ys1 = [circle2.pose.position.y + circle2.scale.x /
           2.0 * m.sin(y) for y in yaw_range]

    # change vector heading vector
    cross_vec = -cross_vec

    # orientation gap between heading angle of parking lot and contact point
    theta2 = m.acos(np.dot(std_vec, cross_vec) /
                    ((np.hypot(std_vec[0], std_vec[1])) *
                     np.hypot(cross_vec[0], cross_vec[1])))

    # theta range for circle 2
    yaw_start = yaw - theta2
    yaw_end = yaw - m.pi / 2
    yaw_range = np.arange(yaw_start, yaw_end, 0.01 *
                          (1.0 if yaw_end > yaw_start else -1.0))

    # Set end point
    end_x = circle1.pose.position.x + \
        circle1.scale.x / 2.0 * \
        m.cos(yaw_range[-1]) - circle1.scale.x / 2.0 * m.cos(yaw)
    end_y = circle1.pose.position.y + \
        circle1.scale.x / 2.0 * \
        m.sin(yaw_range[-1]) - circle1.scale.x / 2.0 * m.sin(yaw)

    xs2 = [circle1.pose.position.x +
           circle1.scale.x / 2.0 * m.cos(y) for y in yaw_range]
    ys2 = [circle1.pose.position.y +
           circle1.scale.x / 2.0 * m.sin(y) for y in yaw_range]

    xs = [start_x] + xs1 + xs2 + [end_x]
    ys = [start_y] + ys1 + ys2 + [end_y]

    cx, cy, cyaw, _, _ = calc_spline_course(xs, ys, 0.01)

    path = Path()
    path.header = Header(None, rospy.Time.now(), "map")

    for i in range(len(cx)):
        quat = quaternion_from_euler(0., 0., cyaw[i])

        p = PoseStamped()
        p.header = Header(None, rospy.Time.now(), "map")
        p.pose.position = Point(cx[i], cy[i], 0.)
        p.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        path.poses.append(p)

    return path


def getTwoCircle(idx):
    global circles_pub

    selected_parking_area = parking_areas[idx]

    h = selected_parking_area.scale.x   # 2.0
    w = selected_parking_area.scale.y   # 0.8

    # ===== circle 1 =====

    circle1 = Marker()

    circle1.header = Header(None, rospy.Time.now(), "map")
    circle1.ns = "0"
    circle1.id = 0
    circle1.type = 3
    circle1.action = 0
    circle1.color = ColorRGBA(0, 0, 1, 0.3)

    center1 = selected_parking_area.position
    _, _, yaw = euler_from_quaternion([
        selected_parking_area.orientation.x,
        selected_parking_area.orientation.y,
        selected_parking_area.orientation.z,
        selected_parking_area.orientation.w
    ])

    cir_center1 = Point(
        center1.x - h * 0.25 * m.cos(yaw) +
        w * 0.5 * m.cos(yaw + m.pi / 2),
        center1.y - h * 0.25 * m.sin(yaw) +
        w * 0.5 * m.sin(yaw + m.pi / 2),
        0
    )

    r1 = w / 2.0

    circle1.pose.position = cir_center1
    circle1.pose.orientation = Quaternion(0, 0, 0, 1)
    circle1.scale = Vector3(r1 * 2.0, r1 * 2.0, 0.1)

    # ===== circle 2 =====
    circle2 = Marker()

    circle2.header = Header(None, rospy.Time.now(), "map")
    circle2.ns = "1"
    circle2.id = 1
    circle2.type = 3
    circle2.action = 0
    circle2.color = ColorRGBA(0, 1, 0, 0.3)

    # Calculate Contact Point (Circle1 - Straight Line)
    arrange = np.arange(0., h, 0.01)

    xs = center1.x + h * 0.5 * \
        m.cos(yaw) + w * m.cos(yaw + m.pi / 2) - \
        arrange * m.cos(yaw + m.pi / 2)
    ys = center1.y + h * 0.5 * \
        m.sin(yaw) + w * m.sin(yaw + m.pi / 2) - \
        arrange * m.sin(yaw + m.pi / 2)
    idx = np.argmin(np.abs(arrange - np.hypot(xs - center1.x, ys - center1.y)))

    cir_center2 = Point(
        xs[idx],
        ys[idx],
        0.0
    )

    r2 = np.hypot(circle1.pose.position.x -
                  cir_center2.x, circle1.pose.position.y - cir_center2.y) - r1

    circle2.pose.position = cir_center2
    circle2.pose.orientation = Quaternion(0, 0, 0, 1)
    circle2.scale = Vector3(r2 * 2.0, r2 * 2.0, 0.1)

    if circles_pub.get_num_connections() > 0:
        msg = MarkerArray()
        msg.markers = [circle1, circle2]
        circles_pub.publish(msg)

    return circle1, circle2, selected_parking_area


if __name__ == "__main__":
    rospy.init_node("horizontal_parking")

    state = OdomState("/odometry/kalman")
    parking_areas = []

    parking_sub = rospy.Subscriber(
        "/parking_areas", MarkerArray, callback=markerCallback)
    circles_pub = rospy.Publisher(
        "/circles", MarkerArray, queue_size=1
    )
    path_pub = rospy.Publisher(
        "/parking_path", Path, queue_size=1
    )

    rospy.wait_for_message("/parking_areas", MarkerArray)

    sleep(3.)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        circle1, circle2, selected_parking_area = getTwoCircle(0)
        path = createPath(circle1, circle2, selected_parking_area)

        path_pub.publish(path)

        r.sleep()
