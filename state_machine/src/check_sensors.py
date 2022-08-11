#!/usr/bin/env python

import rospy
import rosgraph
import rostopic
import numpy as np
import math as m
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from erp42_msgs.msg import *
from map_matching_localization.msg import *
from path_plan.msg import *
from std_msgs.msg import *
# from rtcm_msgs.msg import *


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
        # Sensor("/ublox_gps/rtcm", Message)
    ]

    exceptions = [
        "/move_base_simple/goal",
        "/initialpose",
        "/delete_waypoint",
        "/add_waypoint",
        "/create_global_path",
        "/save_waypoints"
    ]

    master = rosgraph.Master("/rostopic")
    pubs, subs = rostopic.get_topic_list(master)

    for sub in subs:
        try:
            name = sub[0]
            topic_type_full = sub[1]
            topic_type = topic_type_full.split("/")[-1]
            node = sub[2]
            exec("topic_type = %s" % topic_type)

            if not name in exceptions:
                sensors.append(Sensor(name, topic_type))

        except Exception as ex:
            rospy.logwarn("Topic type %s is not defined!" % topic_type_full)

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
