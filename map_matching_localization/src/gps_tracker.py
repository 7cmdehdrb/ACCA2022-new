#!/usr/bin/env python

import rospy
import rospkg
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
        self.state = GPState("/ublox_gps/fix")
        self.track1 = self.loadCSV(
            rospkg.RosPack().get_path("map_matching_localization") + "/data/"
            + rospy.get_param("/gps_tracker/track1", "test.csv")
        )
        self.track2 = self.loadCSV(
            rospkg.RosPack().get_path("map_matching_localization") + "/data/"
            + rospy.get_param("/gps_tracker/track2", "test2.csv")
        )

    def loadCSV(self, file):
        temp = []
        with open(file, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")
            for row in reader:
                time = row[0]
                odom = Point(float(row[1]), float(row[2]), 0.)
                gps = Point(float(row[3]), float(row[4]), 0.)

                temp.append([odom, gps])

        return temp

    def calculatePoint(self):
        min_dist1 = float("inf")
        min_dist2 = float("inf")

        near_odom1 = None
        near_odom2 = None

        x = self.state.x
        y = self.state.y

        for odom, gps in self.track1:
            dist = np.hypot(gps.x - x, gps.y - y)

            if dist < 0.5:
                min_dist1 = dist
                near_odom1 = odom
                break

            if dist < min_dist1:
                min_dist1 = dist
                near_odom1 = odom

        for odom, gps in self.track2:
            dist = np.hypot(gps.x - x, gps.y - y)

            if dist < 0.5:
                min_dist2 = dist
                near_odom2 = odom
                break

            if dist < min_dist2:
                min_dist2 = dist
                near_odom2 = odom

        x = near_odom1.x * (min_dist2 / (min_dist1 + min_dist2)) + \
            near_odom2.x * (min_dist1 / (min_dist1 + min_dist2))

        y = near_odom1.y * (min_dist2 / (min_dist1 + min_dist2)) + \
            near_odom2.y * (min_dist1 / (min_dist1 + min_dist2))

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
    rospy.init_node("gps_tracker")

    gps_odom = GPSOdometry()

    pub = rospy.Publisher("/gps_tracker/odometry", PointStamped, queue_size=1)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        msg = PointStamped()
        msg.header = Header(None, rospy.Time.now(), "map")
        msg.point = gps_odom.calculatePoint()

        pub.publish(msg)

        r.sleep()
