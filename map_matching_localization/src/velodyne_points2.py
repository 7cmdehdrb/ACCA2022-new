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


def pclCallback(msg):
    print(msg.header)
    msg.header.frame_id = "velodyne"
    msg.header.stamp = rospy.Time.now()
    pcl_pub.publish(msg)
    rospy.loginfo("Publishing...")
    

if __name__ == "__main__":
    rospy.init_node("velodyne2")

    pcl_sub = rospy.Subscriber(
        "velodyne_points", PointCloud2, callback=pclCallback)
    pcl_pub = rospy.Publisher("velodyne_points2", PointCloud2, queue_size=5)

    rospy.spin()
