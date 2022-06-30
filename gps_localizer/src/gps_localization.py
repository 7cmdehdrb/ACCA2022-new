#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import NavSatFix
from pyproj import Proj, transform


class GPS_Position(object):
    def __init__(self, x, y, cov):
        self.position = Point(x, y, 0.)
        self.covariance = list(cov)   # 3 x 3

    def transformOdometry(self):
        msg = Odometry()

        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()

        msg.child_frame_id = "base_link"

        msg.pose.pose.position = self.position
        msg.pose.pose.orientation = Quaternion(0., 0., 0., 1.)

        # cov == len 9
        msg.pose.covariance = self.covariance + \
            [0. for i in range(36 - len(self.covariance))]

        return msg


class GPS_Localizer(object):
    def __init__(self):
        self.gps_sub = rospy.Subscriber(
            "/ublox_gps/fix", NavSatFix, callback=self.GpsCallback)
        self.gps_pub = rospy.Publisher(
            "/odometry/gps2map", Odometry, queue_size=5)

        # Map Frame (m)
        self.A1 = [44.077665890002898, -6.5612605426349839]
        self.A2 = [28.891198053973657, -67.680239378889823]
        self.A3 = [-29.312756358060142, -15.256011575710552]

        # UTM (scale: unknown)
        self.B1 = [952084.55943544081, 1944299.9814490867]
        self.B2 = [952068.92547413474, 1944238.9990436761]
        self.B3 = [952011.13217425707, 1944291.7271537485]

        D_utm = m.sqrt((self.B3[0] - self.B2[0])
                       ** 2 + (self.B3[1] - self.B2[1]) ** 2)
        D_map = m.sqrt((self.A3[0] - self.A2[0])
                       ** 2 + (self.A3[1] - self.A2[1]) ** 2)
        self.R = D_map / D_utm

    def GpsCallback(self, msg):
        self.position_coordinate(msg)

    def publishOdometry(self, msg):
        self.gps_pub.publish(msg)

    def position_coordinate(self, GpsMsg):
        latitude = GpsMsg.latitude
        longitude = GpsMsg.longitude
        covariance = GpsMsg.position_covariance

        proj_UTMK = Proj(init='epsg:5178')
        proj_WGS84 = Proj(init='epsg:4326')

        # WGS84 -> UTM-K
        UTM_x, UTM_y = transform(
            proj_WGS84, proj_UTMK, longitude, latitude)

        # distance
        self.D1 = (m.sqrt(
            (UTM_x - self.B1[0]) ** 2 + (UTM_y - self.B1[1]) ** 2)) * self.R
        self.D2 = (m.sqrt(
            (UTM_x - self.B2[0]) ** 2 + (UTM_y - self.B2[1]) ** 2)) * self.R
        self.D3 = (m.sqrt(
            (UTM_x - self.B3[0]) ** 2 + (UTM_y - self.B3[1]) ** 2)) * self.R

        S = (self.A3[0] ** 2 - self.A2[0] ** 2 + self.A3[1] **
             2 - self.A2[1] ** 2 + self.D2 ** 2 - self.D3 ** 2) / 2
        T = (self.A1[0] ** 2 - self.A2[0] ** 2 + self.A1[1] **
             2 - self.A2[1] ** 2 + self.D2 ** 2 - self.D1 ** 2) / 2

        y = ((T * (self.A2[0] - self.A3[0])) - (S * (self.A2[0] - self.A1[0]))) / (((self.A1[1] - self.A2[1])
                                                                                    * (self.A2[0] - self.A3[0])) - ((self.A3[1] - self.A2[1]) * (self.A2[0] - self.A1[0])))
        x = ((y * (self.A1[1] - self.A2[1]) - T) / (self.A2[0] - self.A1[0]))

        # float, float, (3, 3) matrix
        msg = GPS_Position(x, y, covariance).transformOdometry()
        self.publishOdometry(msg)


if __name__ == "__main__":
    rospy.init_node('gps_localizer')

    # main object
    gpsLocal = GPS_Localizer()

    rospy.spin()
