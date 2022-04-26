#!/usr/bin/env python


import rospy
import rospkg
from header import *


if __name__ == "__main__":
    rospy.init_node("test")

    odom = MapMatchingOdometry("odometry/global")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        print(odom.getOdomOnMap())

        rate.sleep()
