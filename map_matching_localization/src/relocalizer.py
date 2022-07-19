#!/usr/bin/env python

from time import sleep
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from hdl_localization.msg import ScanMatchingStatus
from nav_msgs.msg import Odometry
from header import Queue

# Param

odom_topic = rospy.get_param(
    param_name="/relocalizer/odom_topic", default="odometry/kalman")
matching_err_tol = float(rospy.get_param(
    param_name="/relocalizer/matching_err_tol", default=0.1))
inlier_fraction_tol = float(rospy.get_param(
    param_name="/relocalizer/inlier_fraction_tol", default=0.9))


class Relocalizer(object):
    def __init__(self):
        self.init_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, callback=self.odomCallback
        )
        self.status_sub = rospy.Subscriber(
            "/status", ScanMatchingStatus, callback=self.statusCallback)

        self.odom_pose = PoseWithCovarianceStamped()
        self.matching_err_queue = Queue(length=30)

    def statusCallback(self, msg):
        self.matching_err_queue.inputValue(
            self.isTrustable(msg.matching_error, msg.inlier_fraction))

        if self.matching_err_queue.isFalse(30) is True:
            rospy.loginfo("RELOCALIZING...")
            self.init_pub.publish(self.odom_pose)
            self.matching_err_queue.inputValue(True)

    def isTrustable(self, matching_err, inlier_fraction):
        return matching_err <= matching_err_tol and inlier_fraction >= inlier_fraction_tol

    def odomCallback(self, msg):
        self.odom_pose = self.transformOdometryToPoseWithCovarianceStamped(msg)

    def transformOdometryToPoseWithCovarianceStamped(self, odom):
        pose = PoseWithCovarianceStamped()

        pose.header.frame_id = odom.header.frame_id
        pose.header.stamp = rospy.Time.now()

        pose.pose.pose = odom.pose.pose

        return pose


if __name__ == "__main__":
    rospy.init_node("relocalizer")

    relocalizer = Relocalizer()

    rospy.spin()
