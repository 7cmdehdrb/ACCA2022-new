#!/usr/bin/env python

import rospy
import math as m
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




        self.MAP_array = [(44.7440071105957, -22.97344970703125), (57.66763687133789, 15.54519271850586), (68.47551544189453, 63.56060089111328), (-16.925106048583984, 96.37699127197266), (-48.61351013183594, 12.82844352722168), (7.875662326812744, 28.958160400390625)]
        self.UTM_array = [(935792.3273586094, 1915571.9181262662), (935814.0398102237, 1915606.4008861256), (935835.8612522263, 1915650.4362249952), (935760.2116881593, 1915702.183413668), (935710.2458660271, 1915628.1536590266), (935769.1164516942, 1915631.3861677346)]

        # self.MAP_array = [(4.951485633850098, 9.121633529663086), (50.159671783447266, 16.569351196289062), (55.40330505371094, -49.641634979248046), (-11.081011238098144, -68.96293151855468), (-24.29832649230957, -24.148726654052734)]
# 
        # self.UTM_array = [(952238.8559021528, 1943997.5906348864), (952282.4780451846, 1943983.2939213214), (952256.5764822852, 1943922.1215198254), (952188.5662703076, 1943935.060860259), (952197.5051409351, 1943981.0530319982)]
# 


        # school
        # Map Frame (m)
        # self.MAP_array = [(-0.4960020862768175, -0.5760807573118396), (46.652636456447496, -8.785138142668425), (39.851066110081725, -
                                                                                                                #  68.52667876032093), (-36.800244787332694, -56.39848317530497), (-29.25821810981983, -15.20442872770482)]

        # UTM (scale: unknown)
        # self.UTM_array = [(952040.030150764, 1944306.2372952956), (952087.0743343225, 1944297.801344242), (952079.8538559552,
                                                                                                        #    1944238.092975102), (952003.4613295222, 1944250.5698888663), (952011.2476501107, 1944291.8263297458)]
        # k-city

        D_map = m.sqrt((self.MAP_array[0][0] - self.MAP_array[1][0])
                       ** 2 + (self.MAP_array[0][1] - self.MAP_array[1][1]) ** 2)
        D_utm = m.sqrt((self.UTM_array[0][0] - self.UTM_array[1][0])
                       ** 2 + (self.UTM_array[0][1] - self.UTM_array[1][1]) ** 2)
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

        S = (A3[0] ** 2 - A2[0] ** 2 + A3[1] **
             2 - A2[1] ** 2 + D2 ** 2 - D3 ** 2) / 2
        T = (A1[0] ** 2 - A2[0] ** 2 + A1[1] **
             2 - A2[1] ** 2 + D2 ** 2 - D1 ** 2) / 2

        y = ((T * (A2[0] - A3[0])) - (S * (A2[0] - A1[0]))) / (((A1[1] - A2[1])
                                                                * (A2[0] - A3[0])) - ((A3[1] - A2[1]) * (A2[0] - A1[0])))
        x = ((y * (A1[1] - A2[1]) - T) / (A2[0] - A1[0]))

        # float, float, (3, 3) matrix
        msg = GPS_Position(x, y, covariance).transformOdometry()
        self.publishOdometry(msg)


if __name__ == "__main__":
    rospy.init_node('gps_localizer')

    # main object
    gpsLocal = GPS_Localizer()

    rospy.spin()
