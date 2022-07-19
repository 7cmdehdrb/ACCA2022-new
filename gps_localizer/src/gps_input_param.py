
#!/usr/bin/env python

import math as m
import numpy as np
import rospy
import pandas as pd
from pyproj import Proj, transform
import tf
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from time import sleep

'''
<input>
map_coordinate : A1, A2, A3 .... A?
UTM_coordinate : B1, B2, B3, B
distance : Da12, Da23, Da13
ratio : R
'''


class State():
    def __init__(self, gps_param=None):
        self.gps_params = gps_param

    def update(self):
        self.__utm_x, self.__utm_y, self.__map_x, self.__map_y = self.gps_params.handleData()

        return self.__utm_x, self.__utm_y, self.__map_x, self.__map_y


class get_params(object):
    def __init__(self):
        self.gps_sub = rospy.Subscriber(
            "/ublox_gps/fix", NavSatFix, callback=self.GpsCallback)
        self.Odom_sub = rospy.Subscriber(
            "/odom", Odometry, callback=self.OdomCallback)

        self.GpsMsg = NavSatFix()
        self.OdomMsg = Odometry()

    def GpsCallback(self, msg):
        self.GpsMsg = msg

    def OdomCallback(self, msg):
        self.OdomMsg = msg

    def handleData(self):
        self.latitude = self.GpsMsg.latitude
        self.longitude = self.GpsMsg.longitude

        # UTM-K
        proj_UTMK = Proj(init='epsg:5178')
        # WGS1984
        proj_WGS84 = Proj(init='epsg:4326')

        # WGS84 -> UTM-K
        self.UTM_x, self.UTM_y = transform(
            proj_WGS84, proj_UTMK, self.longitude, self.latitude)
        self.odom_x = self.OdomMsg.pose.pose.position.x
        self.odom_y = self.OdomMsg.pose.pose.position.y

        return self.UTM_x, self.UTM_y, self.odom_x, self.odom_y


if __name__ == "__main__":

    rospy.init_node('gps_params')
    get_pa = get_params()
    state = State(get_pa)

    df = pd.DataFrame

    r = rospy.Time.now()
    MAP_x = []
    MAP_y = []
    UTM_x = []
    UTM_y = []

    while not rospy.is_shutdown():
        a = int(input("start:0     stop:1  "))
        if a == 0:
            utm_x = []
            utm_y = []
            map_x = []
            map_y = []

            for i in range(1):
                gx, gy, mx, my = state.update()
                utm_x.append(gx)
                utm_y.append(gy)
                map_x.append(mx)
                map_y.append(my)
            _utm_x = np.mean(utm_x)
            _utm_y = np.mean(utm_y)
            _map_x = np.mean(map_x)
            _map_y = np.mean(map_y)
            MAP_x.append(_map_x)
            MAP_y.append(_map_y)
            UTM_x.append(_utm_x)
            UTM_y.append(_utm_y)
 
        elif a == 1:
            break
        else:
            print("error number")
            pass

    print(MAP_x)
    print(MAP_y)
    print(UTM_x)
    print(UTM_y)
    df = pd.DataFrame(columns=['MAP_x','MAP_y', 'UTM_x', 'UTM_y'])
    df["MAP_x"] = MAP_x
    df["MAP_y"] = MAP_y
    df["UTM_x"] = UTM_x
    df["UTM_y"] = UTM_y

    df.to_csv("k-city_12.csv", index=False)