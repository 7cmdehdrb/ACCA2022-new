#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Point
from hdl_localization.msg import ScanMatchingStatus, HDL_TF


"""

    TO DO: Write Detail

"""


# Param


odom_topic = rospy.get_param(
    param_name="/hdl_tf_node/odom_topic", default="odometry/global")
matching_err_tol = float(rospy.get_param(
    param_name="/hdl_tf_node/matching_err_tol", default=0.05))
inlier_fraction_tol = float(rospy.get_param(
    param_name="/hdl_tf_node/inlier_fraction_tol", default=0.95))


# Utils Functions

def translationToArray(trans):
    return [trans.x, trans.y, trans.z]


def rotationToArray(rot):
    return [rot.x, rot.y, rot.z, rot.w]


def transformPoseToPoseWithCov(pose):
    assert type(pose) == type(PoseStamped())
    poseCov = PoseWithCovarianceStamped()

    poseCov.header = pose.header
    poseCov.pose.pose = pose.pose

    return poseCov


class Queue(object):
    def __init__(self, length=10, init=True):
        self.__array = [init for i in range(length)]

    def inputValue(self, value):
        # assert type(value) == bool

        self.__array.append(value)
        del self.__array[0]

    def count(self, flag):
        return self.__array.count(flag)

    def isTrue(self, threshhold=10):
        return self.count(True) >= threshhold

    def isFalse(self, threshhold=10):
        return self.count(False) >= threshhold


class HDL_State(object):
    def __init__(self):
        self.__status_sub = rospy.Subscriber(
            "/status", ScanMatchingStatus, callback=self.statusCallback)

        self.__matching_error = float("inf")
        self.__inlier_fraction = 0.

    def statusCallback(self, msg):
        self.__matching_error = msg.matching_error
        self.__inlier_fraction = msg.inlier_fraction

    def isTrustable(self):
        return self.__matching_error <= matching_err_tol and self.__inlier_fraction >= inlier_fraction_tol


class OdometryGlobal(object):
    def __init__(self):
        self.__odom = Odometry()

        self.__odom_sub = rospy.Subscriber(
            "/odometry/global", Odometry, callback=self.odomCallback
        )

    def odomCallback(self, msg):
        self.__odom = msg

    def transformOdometryToPose(self):
        pose = PoseStamped()

        pose.header.frame_id = self.__odom.header.frame_id
        pose.header.stamp = rospy.Time(0)
        pose.pose = self.__odom.pose.pose

        return pose
