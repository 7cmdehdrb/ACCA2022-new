#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import math as m
import genpy
from enum import Enum
from random import randint, random
from matplotlib import pyplot as plt
from scipy.spatial import Delaunay
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class Color(Enum):
    RED = 1
    BLUE = 2


class Cone(object):
    def __init__(self, point=Point(), color=Color.RED):
        self.point = point
        self.color = color


class ConeTest(np.ndarray):
    def __init__(self, color=Color.RED):
        super(np.ndarray, self).__init__()

        self.color = color


class ConeTracker(object):
    def __init__(self):
        self.cones = []
        self.createCones()

        self.cones_pub = rospy.Publisher("cones", MarkerArray, queue_size=1)

    def parseNumpy(self):
        res = []

        for cone in self.cones:
            # cone = Cone()
            res.append([cone.point.x, cone.point.y])

        return np.array(res)

    def plotCones(self):
        for cone in self.cones:
            # cone = Cone()
            plt.scatter(cone.point.x, cone.point.y, color="red" if (
                cone.color == Color.RED) else "blue")

    def createCones(self):
        j = 0

        for i in range(0, 10):
            red_cone = Cone(point=Point(i + randint(-10, 10) *
                            0.01, j-(1 + randint(0, 10) * 0.05), 0.), color=Color.RED)

            blue_cone = Cone(point=Point(i + randint(-10, 10)
                             * 0.01, j+(1 + randint(0, 10) * 0.05), 0.), color=Color.BLUE)

            if random() < 0.8:
                self.cones.append(red_cone)
            if random() < 0.8:
                self.cones.append(blue_cone)

            j = m.sin(i * 0.1) * 10

    def plotCones(self):
        for cone in self.cones:
            # cone = Cone()
            plt.scatter(cone.point.x, cone.point.y, color="red" if (
                cone.color == Color.RED) else "blue")

    def publishCones(self):
        msg = MarkerArray()

        for i in range(len(self.cones)):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()

            marker.ns = str(i)
            marker.id = i

            marker.type = 3
            marker.action = 0

            marker.pose.position = self.cones[i].point
            marker.pose.orientation = Quaternion(0., 0., 0., 1)
            marker.scale = Vector3(0.2, 0.2, 0.2)
            marker.color = ColorRGBA(
                1. if (self.cones[i].color == Color.RED) else 0.,
                0.,
                1. if (self.cones[i].color == Color.BLUE) else 0.,
                1.
            )

            marker.lifetime = genpy.Duration(secs=1)

            msg.markers.append(marker)

        self.cones_pub.publish(msg)


if __name__ == "__main__":
    # rospy.init_node("cone_test")

    tracker = ConeTracker()

    tracker.plotCones()

    points = tracker.parseNumpy()
    tri = Delaunay(points)

    print(tri.simplices)

    plt.triplot(points[:, 0], points[:, 1], tri.simplices)
    plt.show()

    # r = rospy.Rate(3.)
    # while not rospy.is_shutdown():
    #     tracker.publishCones()
    #     r.sleep()
