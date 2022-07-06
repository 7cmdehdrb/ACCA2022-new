#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from erp42_msgs.msg import SerialFeedBack


class Sensor(object):
    def __init__(self, topic, msg_type):

        self.topic = topic

        self.sub = rospy.Subscriber(
            topic, msg_type, callback=self.sensorCallback)

        self.is_available = False

    def sensorCallback(self, msg):
        self.is_available = True
        self.sub.unregister()


if __name__ == "__main__":
    rospy.init_node("check_sensor")

    # Add topic and type here
    sensors = [
        Sensor("/imu/data", Imu),
        Sensor("/ublox_gps/fix", NavSatFix),
        Sensor("/erp42_feedback", SerialFeedBack),
        Sensor("/velodyne_points", PointCloud2),
    ]

    r = rospy.Rate(1.)
    while not rospy.is_shutdown():

        txt = "Cannot subscribe "
        cnt = 0

        for sensor in sensors:
            if sensor.is_available is False:
                cnt += 1
                txt += sensor.topic + ", "

        if cnt == 0:
            txt = "Entire Sensors are available"
            rospy.loginfo(txt)
            break

        rospy.loginfo(txt)

        r.sleep()
