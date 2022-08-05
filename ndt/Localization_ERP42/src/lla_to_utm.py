#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from pyproj import Proj, transform

proj_WGS84 = Proj(init='epsg:4326')
proj_UTMK_5186 = Proj(init='epsg:5186')

def callback(data):
    lat=data.latitude
    lon=data.longitude
    print(lat,lon)
    x, y = transform(proj_WGS84, proj_UTMK_5186, lon, lat)
    print(x, y)
    
if __name__ == '__main__':
    rospy.init_node("main_node")
    rate = rospy.Rate(10)
    sub_gnss = rospy.Subscriber("/ublox_gps/fix",NavSatFix, callback, queue_size=1)
    rospy.spin()
    