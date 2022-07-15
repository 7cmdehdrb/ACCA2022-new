#!/usr/bin/env python

import rospy
import math as m
import pandas as pd
import numpy as np
import tf
import csv
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

        msg.header.frame_id = "map"45
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
            new_cov[i][i] = cov[i][i] * m.sqrt(dist)

        new_cov = list(np.reshape(new_cov, newshape=(36, 1)))
        msg.pose.covariance = new_cov
        # print(msg)

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

        data = pd.read_csv("k-city-03_test.csv")
        
        #print(data)
        MAP_array = []
        UTM_array = []
        
        MAP_x = []
        MAP_y = []
        UTM_x = []
        UTM_y = []

        for i in data.MAP_x:
            MAP_x.append(i)
        for i in data.MAP_y:
            MAP_y.append(i)
        for i in data.UTM_x:
            UTM_x.append(i)
        for i in data.UTM_y:
            UTM_y.append(i)

        l = len(UTM_x)
        for i in range(l):
            UTM_array.append((UTM_x[i], UTM_y[i]))
        print(UTM_array)
        for i in range(l):
            MAP_array.append((MAP_x[i], MAP_y[i]))
        print(MAP_array)

        self.MAP_array = MAP_array
        self.UTM_array = UTM_array
        # print(self.MAP_array)
        D_map = m.sqrt((self.MAP_array[0][0] - self.MAP_array[1][0]) ** 2 + (self.MAP_array[0][1] - self.MAP_array[1][1]) ** 2)
        D_utm = m.sqrt((self.UTM_array[0][0] - self.UTM_array[1][0]) ** 2 + (self.UTM_array[0][1] - self.UTM_array[1][1]) ** 2)
        print(D_map)
        self.R = D_map / D_utm
        print(self.R)
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

        d_array = []
        min_d = []

        for i in self.UTM_array:
            d = m.sqrt((UTM_x - i[0]) ** 2 + (UTM_y - i[1]) ** 2)
            d_array.append(d)

        d_pop = list(d_array)
        d_pop.sort()

        for i in range(3):
            pop = d_pop.pop(0)
            min_d.append(pop)

        first = d_array.index(min_d[0])
        second = d_array.index(min_d[1])
        third = d_array.index(min_d[2])

        print(first, second, third)

        A1 = self.MAP_array[first]
        A2 = self.MAP_array[second]
        A3 = self.MAP_array[third]

        B1 = self.UTM_array[first]
        B2 = self.UTM_array[second]
        B3 = self.UTM_array[third]

        # distance
        D1 = (m.sqrt(
            (UTM_x - B1[0]) ** 2 + (UTM_y - B1[1]) ** 2)) * self.R
        D2 = (m.sqrt(
            (UTM_x - B2[0]) ** 2 + (UTM_y - B2[1]) ** 2)) * self.R
        D3 = (m.sqrt(
            (UTM_x - B3[0]) ** 2 + (UTM_y - B3[1]) ** 2)) * self.R
        print(D1, D2, D3)
        S = (A3[0] ** 2 - A2[0] ** 2 + A3[1] ** 2 - A2[1] ** 2 + D2 ** 2 - D3 ** 2) / 2
        T = (A1[0] ** 2 - A2[0] ** 2 + A1[1] ** 2 - A2[1] ** 2 + D2 ** 2 - D1 ** 2) / 2

        y = ((T * (A2[0] - A3[0])) - (S * (A2[0] - A1[0]))) / (((A1[1] - A2[1]) * (A2[0] - A3[0])) - ((A3[1] - A2[1]) * (A2[0] - A1[0])))
        x = ((y * (A1[1] - A2[1]) - T) / (A2[0] - A1[0]))

        # float, float, (3, 3) matrix
        msg = GPS_Position(x, y, covariance).transformOdometry()
        self.publishOdometry(msg)


if __name__ == "__main__":
    rospy.init_node('gps_localizer')

    # main object
    gpsLocal = GPS_Localizer()

    rospy.spin()