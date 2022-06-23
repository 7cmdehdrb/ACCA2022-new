#!/usr/bin/env python


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
from cubic_spline_planner import calc_spline_course


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
        self.path_pub = rospy.Publisher("cone_path", Path, queue_size=1)

        self.xs = []
        self.ys = []

    def createVirtualCones(self):
        j = 0

        for i in range(0, 50):
            red = Cone(
                [i + randint(-10, 10) * 0.01, j-(1 + randint(0, 10) * 0.05)]
            )

            red.color = Color.RED
            self.cones.append(red)

            blue = Cone(
                [i + randint(-10, 10) * 0.01, j+(1 + randint(0, 10) * 0.05)]
            )

            blue.color = Color.BLUE
            self.cones.append(blue)

            j = 16 * (m.sin(i * 0.1)) ** 3
            # j = m.sin(i * 0.1) * 10

    def plotCones(self):
        for cone in self.cones:
            plt.scatter(cone[0], cone[1], color="red" if (
                cone.color == Color.RED) else "blue")

    def plotDelaunay(self, ids=[1, 2, 3]):
        p0 = self.cones[ids[0]]
        p1 = self.cones[ids[1]]
        p2 = self.cones[ids[2]]

        if p0.color != p1.color:
            plt.plot([p0[0], p1[0]], [p0[1], p1[1]], color="black")
            x = (p0[0] + p1[0]) / 2.
            y = (p0[1] + p1[1]) / 2.

            self.centers.append(Cone([x, y], color=Color.CENTER))

            plt.scatter(x, y, s=5, color="red")

        if p1.color != p2.color:
            x = (p1[0] + p2[0]) / 2.
            y = (p1[1] + p2[1]) / 2.

            self.centers.append(Cone([x, y], color=Color.CENTER))

            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color="black")
            plt.scatter(x, y, s=5, color="red")

        if p0.color != p2.color:
            x = (p0[0] + p2[0]) / 2.
            y = (p0[1] + p2[1]) / 2.

            self.centers.append(Cone([x, y], color=Color.CENTER))

            plt.plot([p0[0], p2[0]], [p0[1], p2[1]], color="black")
            plt.scatter(x, y, s=5, color="red")

    def append(self, cone):
        self.cones.append(cone)
        return self.cones

    def getDistance(self, p1, p2):
        dist = m.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
        return dist

    def removeDuplicate(self):
        n = np.unique(self.centers, axis=0)
        self.centers = n

    def get_ordered_list(self):
        self.removeDuplicate()
        sorted_point = sorted(
            self.centers, key=lambda e: self.getDistance(e, Cone([0., 0.])))

        self.xs = []
        self.ys = []

        for cone in sorted_point:
            print(cone)
            self.xs.append(cone[0])
            self.ys.append(cone[1])

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

    def publishCones(self):
        msg = MarkerArray()

        for i, cone in enumerate(self.cones):
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

            marker.lifetime = genpy.Duration(secs=1)

            msg.markers.append(marker)

        self.cones_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("cone_tracker")

    cones = Cones()
    cones.createVirtualCones()
    cones.plotCones()

    tri = Delaunay(cones.cones)

    for ids in tri.simplices:
        cones.plotDelaunay(ids)

    # plt.show()

    cones.get_ordered_list()
    cx, cy, cyaw, _, _ = calc_spline_course(
        cones.xs[:], cones.ys[:], ds=0.05)

    r = rospy.Rate(3.)
    while not rospy.is_shutdown():
        cones.publishCones()
        cones.publishPath(cx, cy, cyaw)
        r.sleep()
