#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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

        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        msg.child_frame_id = "base_link"

        msg.pose.pose.position = self.position
        msg.pose.pose.orientation = Quaternion(0., 0., 0., 1.)

        dist = measure(37.4966977, 126.9575288,
                       37.4966977 + 1, 126.9575288)

        cov = np.reshape(np.array(self.covariance), newshape=(3, 3))
        new_cov = np.array([
            [0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0.],
        ])

        for i in range(2):
            new_cov[i][i] = cov[i][i] * m.sqrt(dist) + 0.05

        new_cov = list(np.reshape(new_cov, newshape=(36, 1)))
        msg.pose.covariance = new_cov

        return msg


def measure(lat1, lon1, lat2, lon2):
    R = 6378.137
    dLat = lat2 * m.pi / 180 - lat1 * m.pi / 180
    dLon = lon2 * m.pi / 180 - lon1 * m.pi / 180
    a = m.sin(dLat/2) * m.sin(dLat/2) + \
        m.cos(lat1 * m.pi / 180) * m.cos(lat2 * m.pi / 180) * \
        m.sin(dLon/2) * m.sin(dLon/2)
    c = 2 * m.atan2(m.sqrt(a), m.sqrt(1-a))
    d = R * c
    return d * 1000


class GPS_Localizer(object):
    def __init__(self):
        self.gps_sub = rospy.Subscriber(
            "/ublox_gps/fix", NavSatFix, callback=self.GpsCallback)
        self.gps_pub = rospy.Publisher(
            "/odometry/gps", Odometry, queue_size=5)

        # Map Frame (m)
        self.A1 = [-12.008216791152954, 1.8138237690925598]
        self.A2 = [51.58015068054199, -3.5253489112854002]
        self.A3 = [-3.240585916042328, -71.910366439819342]

        # UTM (scale: unknown)
        self.B1 = [952220.27949039731, 1943998.3930866024]
        self.B2 = [952274.35303833662, 1943964.6235879373]
        self.B3 = [952194.09346341831, 1943928.4288452363]

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
