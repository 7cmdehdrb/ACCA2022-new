#!/usr/bin/env python

import rospy
import numpy as np
import math as m
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pyproj import*


def calculateDistance(p1, p2):
    return m.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


class GPSOdometry(object):
    def __init__(self):

        self.gps = Proj(init="epsg:4326")   # lat, log
        self.tm = Proj(init="epsg:5186")    # m

        self.ublox = Ublox()

        self.x = 0.
        self.y = 0.

        self.last_position = Point()

    def transformGPStoTM(self):
        x, y = transform(p1=self.gps, p2=self.tm,
                         x=self.ublox.log, y=self.ublox.lat)
        current_point = Point(x, y, 0.)
        dis = calculateDistance(self.last_position, current_point)
        self.last_position = current_point

        return dis


class Ublox(object):
    def __init__(self):
        self.sub = rospy.Subscriber(
            "/ublox_gps/fix", NavSatFix, callback=self.gpsCallback)

        self.lat = 0.
        self.log = 0.

    def gpsCallback(self, msg):
        # msg = NavSatFix()

        self.once = False
        self.lat = msg.latitude
        self.log = msg.longitude
        self.cov = msg.position_covariance
