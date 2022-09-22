#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from tf.transformations import *
from abc import *
from pyproj import *
import csv
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *


class GPSOdometry(object):
    def __init__(self):
        self.data = []

        self.state = GPState("/ublox_gps/fix")
        self.loadCSV(
            "/home/acca/catkin_ws/src/ACCA2022-new/map_matching_localization/data/test2.csv")

    def loadCSV(self, file):
        with open(file, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")
            for row in reader:
                time = row[0]
                odom = Point(float(row[1]), float(row[2]), 0.)
                gps = Point(float(row[3]), float(row[4]), 0.)

                self.data.append([odom, gps])

    def calculatePoint(self):
        near_odom1 = None
        near_odom2 = None

        min_dist = float("inf")
        second_dist = float("inf")

        x = self.state.x
        y = self.state.y

        for odom, gps in self.data:
            dist = np.hypot(gps.x - x, gps.y - y)

            if dist < min_dist:
                second_dist = min_dist
                min_dist = dist

                near_odom2 = near_odom1
                near_odom1 = odom

            elif dist < second_dist:
                second_dist = dist

                near_odom2 = odom

        print(near_odom1, near_odom2)

        x = near_odom1.x * (second_dist / (min_dist + second_dist)) + \
            near_odom2.x * (min_dist / (min_dist + second_dist))

        y = near_odom1.y * (second_dist / (min_dist + second_dist)) + \
            near_odom2.y * (min_dist / (min_dist + second_dist))

        return Point(x, y, 0.)


class GPState(object):
    def __init__(self, topic="/ublox_gps/fix"):
        self.sub = rospy.Subscriber(topic, NavSatFix, callback=self.callback)

        self.gps = Proj(init="epsg:4326")   # lat, log
        self.tm = Proj(init="epsg:2097")    # m

        self.x = 0.
        self.y = 0.

    def callback(self, msg):
        x, y = transform(p1=self.gps, p2=self.tm,
                         x=msg.longitude, y=msg.latitude)

        self.x = x
        self.y = y


if __name__ == "__main__":
    rospy.init_node("test")

    gps_odom = GPSOdometry()

    pub = rospy.Publisher("/ublox_gps/odometry", PointStamped, queue_size=1)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        msg = PointStamped()
        msg.header = Header(None, rospy.Time.now(), "map")
        msg.point = gps_odom.calculatePoint()

        pub.publish(msg)

        r.sleep()
