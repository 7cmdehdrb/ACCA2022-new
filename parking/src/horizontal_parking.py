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
from parking_area import ParkingArea
from cubic_spline_planner import calc_spline_course
from reeds_shepp_path_planning import reeds_shepp_path_planning
from enum import Enum
from time import sleep

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State, OdomState
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)


class HorizontalParking(Enum):
    Straight = 0
    Reverse = 1
    Home = 2
    Final = 3
    End = 4
    Wait = 5


def wait_for_stop(duration):
    global current_time, last_time, r, msg, cmd_pub

    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()

        if dt > duration:
            last_time = current_time
            return 0

        msg.Speed = int(0)
        msg.brake = 120
        cmd_pub.publish(msg)
        r.sleep()


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


def createPath(circle1, circle2, selected_parking_area, state):
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

    # Set start point : left side of green circlr
    start_x = circle2.pose.position.x + \
        circle2.scale.x / 2.0 * \
        m.cos(yaw_range[0]) + (circle2.scale.x / 2.0 + 0.0) * m.cos(yaw)
    start_y = circle2.pose.position.y + \
        circle2.scale.x / 2.0 * \
        m.sin(yaw_range[0]) + (circle2.scale.x / 2.0 + 0.0) * m.sin(yaw)

    # +n m of start point
    start_x2 = circle2.pose.position.x + \
        circle2.scale.x / 2.0 * \
        m.cos(yaw_range[0]) + (circle2.scale.x / 2.0 + 1.0) * m.cos(yaw)
    start_y2 = circle2.pose.position.y + \
        circle2.scale.x / 2.0 * \
        m.sin(yaw_range[0]) + (circle2.scale.x / 2.0 + 1.0) * m.sin(yaw)

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

    # Set end point : +n m(relative with car height) of end point of parking lot
    fs = 1.5
    end_x = selected_parking_area.position.x + (selected_parking_area.scale.x / 2.0) * \
        m.cos(yaw + m.pi) - (1.040 / 2.0) * m.cos(yaw + m.pi) * fs
    end_y = selected_parking_area.position.y + (selected_parking_area.scale.y / 2.0) * \
        m.sin(yaw + m.pi) - (1.040 / 2.0) * m.sin(yaw + m.pi) * fs

    end_x2 = selected_parking_area.position.x + (selected_parking_area.scale.x / 2.0) * \
        m.cos(yaw) - (1.040 / 2.0) * m.cos(yaw) * fs
    end_y2 = selected_parking_area.position.y + (selected_parking_area.scale.y / 2.0) * \
        m.sin(yaw) - (1.040 / 2.0) * m.sin(yaw) * fs

    xs2 = [circle1.pose.position.x +
           circle1.scale.x / 2.0 * m.cos(y) for y in yaw_range]
    ys2 = [circle1.pose.position.y +
           circle1.scale.x / 2.0 * m.sin(y) for y in yaw_range]

    _, _, gyaw = euler_from_quaternion(
        [circle1.pose.orientation.x, circle1.pose.orientation.y, circle1.pose.orientation.z, circle1.pose.orientation.w])

    scx, scy, scyaw, _, _ = reeds_shepp_path_planning(
        sx=state.x,
        sy=state.y,
        syaw=state.yaw,
        gx=start_x,
        gy=start_y,
        gyaw=gyaw,
        maxc=0.1,
        step_size=0.05
    )

    scx2, scy2, scyaw2, _, _ = calc_spline_course(
        [start_x, start_x2], [start_y, start_y2], ds=0.01)

    xs = [start_x2] + [start_x] + xs1 + xs2 + [end_x]
    ys = [start_y2] + [start_y] + ys1 + ys2 + [end_y]

    gcx, gcy, gcyaw, _, _ = calc_spline_course(xs, ys, 0.01)

    hcx, hcy, hcyaw, _, _ = calc_spline_course(
        [end_x, end_x2], [end_y, end_y2], 0.01)

    fcx, fcy, fcyaw, _, _ = calc_spline_course(
        [end_x2, selected_parking_area.position.x],
        [end_y2, selected_parking_area.position.y],
        0.01
    )

    straight_path = PathResponse(
        None,
        None,
        None,
        scx + scx2,
        scy + scy2,
        scyaw + scyaw2
    )

    reverse_path = PathResponse(
        None,
        None,
        None,
        gcx,
        gcy,
        gcyaw
    )

    home_path = PathResponse(
        None,
        None,
        None,
        hcx,
        hcy,
        hcyaw
    )

    final_path = PathResponse(
        None,
        None,
        None,
        fcx,
        fcy,
        fcyaw
    )

    cx = scx + gcx + hcx + fcx
    cy = scy + gcy + hcy + fcy
    cyaw = scyaw + gcyaw + hcyaw + fcyaw

    path = Path()
    path.header = Header(None, rospy.Time.now(), "map")

    for i in range(len(cx)):
        quat = quaternion_from_euler(0., 0., cyaw[i])

        p = PoseStamped()
        p.header = Header(None, rospy.Time.now(), "map")
        p.pose.position = Point(cx[i], cy[i], 0.)
        p.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        path.poses.append(p)

    return straight_path, reverse_path, home_path, final_path, path


def getTwoCircle(idx):
    global circles_pub

    selected_parking_area = parking_areas[idx]

    h = selected_parking_area.scale.x   # 2.0
    w = selected_parking_area.scale.y   # 0.8
    reverse_threshold = 0.05                # min: 0, max : 0.25

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

    r1 = w * 0.5

    cir_center1 = Point(
        center1.x - h * reverse_threshold * m.cos(yaw) +
        r1 * m.cos(yaw + m.pi / 2),
        center1.y - h * reverse_threshold * m.sin(yaw) +
        r1 * m.sin(yaw + m.pi / 2),
        0
    )

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
        m.cos(yaw) + r1 * 2 * m.cos(yaw + m.pi / 2) - \
        arrange * m.cos(yaw + m.pi / 2)
    ys = center1.y + h * 0.5 * \
        m.sin(yaw) + r1 * 2 * m.sin(yaw + m.pi / 2) - \
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
    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)
    test_pub = rospy.Publisher(
        "/test", PoseStamped, queue_size=1
    )

    horizontal_parking_state = HorizontalParking.Straight
    target_idx = 0
    stanley = Stanley()

    last_time = rospy.Time.now()
    current_time = rospy.Time.now()

    rospy.wait_for_message("/parking_areas", MarkerArray)

    sleep(3.)

    idx = 0

    circle1, circle2, selected_parking_area = getTwoCircle(idx)
    spath, rpath, hpath, fpath, path = createPath(
        circle1, circle2, selected_parking_area, state)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        msg = ControlMessage()

        if horizontal_parking_state == HorizontalParking.Straight:

            print("Straight", target_idx)

            msg.Speed = int(7)
            di, target_idx = stanley.stanley_control(
                state=state,
                cx=spath.cx,
                cy=spath.cy,
                cyaw=spath.cyaw,
                last_target_idx=target_idx,
                reverse=False
            )

            di = np.clip(di, -m.radians(30), m.radians(30))
            msg.Steer = m.degrees(-di)
            msg.brake = 0
            msg.Gear = 2

            car_vec = np.array([
                m.cos(state.yaw), m.sin(state.yaw)
            ])
            position_vec = np.array(
                [spath.cx[-1] - state.x, spath.cy[-1] - state.y]
            )

            theta = m.acos(np.dot(car_vec, position_vec) /
                           (np.hypot(car_vec[0], car_vec[1]) * np.hypot(position_vec[0], position_vec[1])))

            if target_idx >= len(spath.cx) - 5 and abs(theta) >= m.pi / 2.0:
                horizontal_parking_state = HorizontalParking.Reverse
                target_idx = 0

                # stanley.setHdrRatio(1.0)
                # stanley.setCGain(0.5)

                wait_for_stop(5)

        elif horizontal_parking_state == HorizontalParking.Reverse:

            print("Reverse", target_idx)

            msg.Speed = int(5)
            di, target_idx = stanley.stanley_control(
                state=state,
                cx=rpath.cx,
                cy=rpath.cy,
                cyaw=rpath.cyaw,
                last_target_idx=target_idx,
                reverse=True
            )

            di = np.clip(di, -m.radians(30), m.radians(30))
            msg.Steer = m.degrees(di)
            msg.brake = 0
            msg.Gear = 0

            car_vec = np.array([
                m.cos(state.yaw + m.pi), m.sin(state.yaw + m.pi)
            ])
            position_vec = np.array(
                [rpath.cx[-1] - state.x, rpath.cy[-1] - state.y]
            )

            theta = m.acos(np.dot(car_vec, position_vec) /
                           (np.hypot(car_vec[0], car_vec[1]) * np.hypot(position_vec[0], position_vec[1])))

            if target_idx >= len(rpath.cx) - 5 and abs(theta) >= m.pi / 2.0:
                horizontal_parking_state = HorizontalParking.Home
                target_idx = 0

                wait_for_stop(5)

        elif horizontal_parking_state == HorizontalParking.Home:

            print("Home", target_idx, len(hpath.cx))

            msg.Speed = int(5)
            di, target_idx = stanley.stanley_control(
                state=state,
                cx=hpath.cx,
                cy=hpath.cy,
                cyaw=hpath.cyaw,
                last_target_idx=target_idx,
                reverse=False
            )

            di = np.clip(di, -m.radians(30), m.radians(30))
            msg.Steer = m.degrees(-di)
            msg.brake = 0
            msg.Gear = 2

            car_vec = np.array([
                m.cos(state.yaw), m.sin(state.yaw)
            ])
            position_vec = np.array(
                [hpath.cx[-1] - state.x, hpath.cy[-1] - state.y]
            )

            theta = m.acos(np.dot(car_vec, position_vec) /
                           (np.hypot(car_vec[0], car_vec[1]) * np.hypot(position_vec[0], position_vec[1])))

            if target_idx >= len(hpath.cx) - 5 and abs(theta) >= m.pi / 2.0:
                horizontal_parking_state = HorizontalParking.Final
                target_idx = 0

                wait_for_stop(5)

        elif horizontal_parking_state == HorizontalParking.Final:

            print("Final", target_idx)

            msg.Speed = int(1)
            di, target_idx = stanley.stanley_control(
                state=state,
                cx=fpath.cx,
                cy=fpath.cy,
                cyaw=fpath.cyaw,
                last_target_idx=target_idx,
                reverse=True
            )

            di = np.clip(di, -m.radians(30), m.radians(30))
            msg.Steer = m.degrees(di)
            msg.Gear = 0

            car_vec = np.array([
                m.cos(state.yaw + m.pi), m.sin(state.yaw + m.pi)
            ])
            position_vec = np.array(
                [fpath.cx[-1] - state.x, fpath.cy[-1] - state.y]
            )

            theta = m.acos(np.dot(car_vec, position_vec) /
                           (np.hypot(car_vec[0], car_vec[1]) * np.hypot(position_vec[0], position_vec[1])))

            if target_idx >= len(fpath.cx) - 5 and abs(theta) >= m.pi / 2.0:
                horizontal_parking_state = HorizontalParking.End
                target_idx = 0

                wait_for_stop(5)

        elif horizontal_parking_state == HorizontalParking.End:
            msg.Speed = int(0)
            msg.brake = 100

        else:
            rospy.logfatal("Invalid Horizontal Parking State!")

        path_pub.publish(path)
        cmd_pub.publish(msg)

        r.sleep()
