#!/usr/bin/env python

import rospy
import math as m
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from pyproj import *


def calculateDistance(p1, p2):
    return m.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


class GPSOdometry(object):
    def __init__(self):
        self.odom_pub = rospy.Publisher(
            "/odometry/gps", Odometry, queue_size=1)

        self.gps = Proj(init="epsg:4326")   # lat, log
        self.tm = Proj(init="epsg:2097")    # m

        self.ublox = Ublox()
        self.xsens = Xsens()

        self.x = 0.
        self.y = 0.

        self.last_position = Point()

    def transformGPStoTM(self):
        x, y = transform(p1=self.gps, p2=self.tm,
                         x=self.ublox.log, y=self.ublox.lat)
        current_point = Point(x, y, 0.)

        dis = calculateDistance(self.last_position, current_point)

        if dis < 10.:
            self.x += dis * m.cos(self.xsens.yaw)
            self.y -= dis * m.sin(self.xsens.yaw)

        self.last_position = current_point

        res = Point(self.x, self.y, 0.)
        print(res)

        return res

    def publishOdometry(self):
        msg = Odometry()

        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()

        msg.child_frame_id = "base_link"

        msg.pose.pose.position = self.transformGPStoTM()
        msg.pose.pose.orientation = Quaternion(0., 0., 0., 1.)

        self.odom_pub.publish(msg)


class Ublox(object):
    def __init__(self):
        self.sub = rospy.Subscriber(
            "/ublox_gps/fix", NavSatFix, callback=self.gpsCallback)

        self.lat = 0.
        self.log = 0.

    def gpsCallback(self, msg):
        # msg = NavSatFix()
        self.lat = msg.latitude
        self.log = msg.longitude


class Xsens(object):
    def __init__(self):
        self.sub = rospy.Subscriber(
            "/imu/data", Imu, callback=self.xsensCallback
        )

        self.init_yaw = None
        self.yaw = 0.

    def xsensCallback(self, msg):
        quat = msg.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        if self.init_yaw is None:
            self.init_yaw = yaw

        self.yaw = self.init_yaw - yaw


if __name__ == "__main__":
    rospy.init_node("gps_odometry")

    # ublox = Ublox()
    odom = GPSOdometry()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        odom.publishOdometry()
        r.sleep()
