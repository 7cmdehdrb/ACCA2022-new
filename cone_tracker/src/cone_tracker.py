#!/usr/bin/env python

import os
import sys
from time import sleep
import rospy
import rospkg
import numpy as np
import math as m
import genpy
from tf.transformations import quaternion_from_euler
from enum import Enum
from random import randint, random
from matplotlib import pyplot as plt
from scipy.spatial import Delaunay
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from erp42_control.msg import ControlMessage
from cubic_spline_planner import calc_spline_course
from dynamic_window_approach import dwa_control, motion, calc_target_index, Config

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)


class Color(Enum):
    NONE = 0
    RED = 1
    BLUE = 2
    CENTER = 3


class Cone(np.ndarray):
    def __new__(cls, shape, *args, **kwargs):
        if "color" in kwargs.keys():
            cls.color = kwargs["color"]
        else:
            cls.color = Color.NONE

        obj = np.asarray(shape).view(cls)
        return obj


class Cones(object):
    def __init__(self):
        self.cones = []
        self.centers = []

        self.cones_pub = rospy.Publisher("cones", MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher("cone_path", Path, queue_size=10)

        self.xs = []
        self.ys = []

    def createVirtualCones(self):
        j = 0

        for i in range(0, 120):
            red = Cone(
                [i + randint(-10, 10) * 0.01, j-(2 + randint(0, 10) * 0.05)]
            )

            red.color = Color.RED
            self.cones.append(red)

            blue = Cone(
                [i + randint(-10, 10) * 0.01, j+(2 + randint(0, 10) * 0.05)]
            )

            blue.color = Color.BLUE
            self.cones.append(blue)

            j = 16 * (m.sin(i * 0.1)) ** 3
            # j = m.sin(i * 0.1) * 10

        return self.cones

    def getDelaunayCenters(self, cones, ids):
        res = []

        p0 = cones[ids[0]]
        p1 = cones[ids[1]]
        p2 = cones[ids[2]]

        if p0.color != p1.color:
            x = (p0[0] + p1[0]) / 2.
            y = (p0[1] + p1[1]) / 2.

            res.append(Cone([x, y], color=Color.CENTER))

        if p1.color != p2.color:
            x = (p1[0] + p2[0]) / 2.
            y = (p1[1] + p2[1]) / 2.

            res.append(Cone([x, y], color=Color.CENTER))

        if p0.color != p2.color:
            x = (p0[0] + p2[0]) / 2.
            y = (p0[1] + p2[1]) / 2.

            res.append(Cone([x, y], color=Color.CENTER))

        return res

    def getAllCenters(self, cones, state):
        centers = []
        tri = Delaunay(cones)

        for ids in tri.simplices:
            center = self.getDelaunayCenters(cones, ids)
            centers += center

        centers = self.get_ordered_list(centers, state)

        return centers

    def getPathFromCenters(self, centers):
        centers = np.array(centers)

        xs = centers[:, 0]
        ys = centers[:, 1]

        cx, cy, cyaw, _, _ = calc_spline_course(
            xs[:], ys[:], ds=0.05)

        return cx, cy, cyaw

    def getDistance(self, p1, p2):
        dist = m.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
        return dist

    def removeDuplicate(self, centers):
        n = np.unique(centers, axis=0)
        return n

    def get_ordered_list(self, centers, state):
        centers = self.removeDuplicate(centers)
        sorted_point = sorted(
            centers, key=lambda e: self.getDistance(e, np.array([state.x, state.y])))

        return sorted_point

    def publishPath(self, cx, cy, cyaw):
        path = Path()

        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for i in range(len(cx)):
            pose = PoseStamped()

            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            quat = quaternion_from_euler(0., 0., cyaw[i])

            pose.pose.position = Point(cx[i], cy[i], 0.)
            pose.pose.orientation = Quaternion(
                quat[0], quat[1], quat[2], quat[3])

            path.poses.append(pose)

        self.path_pub.publish(path)

    def publishCones(self, cones):
        msg = MarkerArray()

        for i, cone in enumerate(cones):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()

            marker.ns = str(i)
            marker.id = i

            marker.type = 3
            marker.action = 0

            marker.pose.position = Point(cone[0], cone[1], 0.)  # cone.point
            marker.pose.orientation = Quaternion(0., 0., 0., 1)
            marker.scale = Vector3(0.2, 0.2, 0.2)
            marker.color = ColorRGBA(
                1. if (cone.color == Color.RED) else 0.,
                0.,
                1. if (cone.color == Color.BLUE) else 0.,
                1.
            )

            marker.lifetime = genpy.Duration(secs=2.)

            msg.markers.append(marker)

        self.cones_pub.publish(msg)

    def detectCones(self, state, r=10.):
        res = []

        car = np.array([state.x, state.y])

        for cone in self.cones:
            dist = self.getDistance(car, cone)

            if dist < r:
                cone_vec = np.array([cone[0] - car[0], cone[1] - car[1]])
                car_vec = np.array([m.cos(state.yaw), m.sin(state.yaw)])

                if np.dot(car_vec, cone_vec) > 0:
                    res.append(cone)

        return res


if __name__ == "__main__":
    rospy.init_node("cone_tracker")

    target_idx = 0
    last_idx = 0
    length = 0

    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)

    state = State()
    stanley = Stanley()

    cones = Cones()
    cones.createVirtualCones()

    r2 = rospy.Rate(1.)
    for i in range(10):
        cones.publishCones(cones.cones)
        r2.sleep()

    r = rospy.Rate(10.)
    while not rospy.is_shutdown():
        detected_cone = cones.detectCones(state, 10.)
        centers = cones.getAllCenters(detected_cone, state)
        cx, cy, cyaw = cones.getPathFromCenters(centers)

        l = len(cx)
        if l != length:
            length = l
            target_idx = 1

        if target_idx == l:
            continue

        di, target_idx = stanley.stanley_control(
            state, cx, cy, cyaw, target_idx)

        print(target_idx)

        msg = ControlMessage()
        msg.Speed = 3.
        msg.Steer = -m.degrees(di)

        cmd_pub.publish(msg)
        cones.publishPath(cx, cy, cyaw)
        cones.publishCones(detected_cone)

        r.sleep()
