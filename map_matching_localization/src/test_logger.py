#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from tf.transformations import *
from abc import *
from pyproj import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *


class Tracker():
    __metaclass__ = ABCMeta

    data = Point()
    datas = []
    subscriber = None

    @abstractmethod
    def callback(self, msg):
        pass

    @abstractmethod
    def logging(self):
        pass


class GPSTracker(Tracker):
    def __init__(self, topic="/ublox_gps/fix"):
        super(GPSTracker, self).__init__()

        self.subscriber = rospy.Subscriber(
            topic, NavSatFix, callback=self.callback)

        self.gps = Proj(init="epsg:4326")   # lat, log
        self.tm = Proj(init="epsg:2097")    # m

    def callback(self, msg):
        x, y = transform(p1=self.gps, p2=self.tm,
                         x=msg.longitude, y=msg.latitude)

        self.data = Point(x, y, 0.)

        return super(GPSTracker, self).callback(msg)

    def logging(self):
        self.datas.append(self.data)
        return super(GPSTracker, self).logging()


class OdometryTracker(Tracker):
    def __init__(self, topic="/ndt_mathcing/ndt_pose"):
        super(OdometryTracker, self).__init__()

        self.subscriber = rospy.Subscriber(
            topic, Odometry, callback=self.callback)

    def callback(self, msg):
        self.data = msg.pose.pose.position
        return super(OdometryTracker, self).callback(msg)

    def logging(self):
        self.datas.append(self.data)
        return super(OdometryTracker, self).logging()


if __name__ == "__main__":
    rospy.init_node("test")

    odom = OdometryTracker("/ndt_matching/ndt_pose")
    gps = GPSTracker("/ublox_gps/fix")
    file = "/home/acca/catkin_ws/src/ACCA2022-new/map_matching_localization/data/test2.csv"

    odom_last_data = odom.data

    with open(file, 'w') as csvfile:
        r = rospy.Rate(5)
        while not rospy.is_shutdown():

            data = odom.data
            distance = np.hypot(odom_last_data.x - data.x,
                                odom_last_data.y - data.y)

            if distance > 1.0:
                text = ""
                text += str(rospy.Time.now().to_sec())
                text += ","
                text += str(odom.data.x)
                text += ","
                text += str(odom.data.y)
                text += ","
                text += str(gps.data.x)
                text += ","
                text += str(gps.data.y)
                text += "\n"

                print(text)

                csvfile.write(text)
                odom_last_data = data

            r.sleep()
