#!/usr/bin/env python


import rospy
import rospkg
import math as m
import numpy as np
import time as t
import threading
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from point_cloud import array_to_xyz_pointcloud2f


if __name__ == "__main__":
    rospy.init_node("tf_sub")

    tf_sub = tf.TransformListener()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():

        try:
            data = tf_sub.lookupTransform(
                "velodyne", "base_link", rospy.Time(0))
            print(data)
        except Exception as ex:
            rospy.logwarn(ex)
        finally:
            r.sleep()

        if tf_sub.canTransform("velodyne", "base_link", rospy.Time(0)):
            pass

        else:
            rospy.logwarn("ERROR")

        r.sleep()
