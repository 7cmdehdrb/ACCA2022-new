#!/usr/bin/env python


import rospy
import numpy as np
import tf
from tf.transformations import *
import threading
from header import Queue
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from hdl_localization.msg import ScanMatchingStatus, HDL_TF


# Param

hdl_topic = rospy.get_param(
    param_name="/hdl_tf_node/hdl_topic", default="hdl_tf")
matching_err_tol = float(rospy.get_param(
    param_name="/hdl_tf_node/matching_err_tol", default=0.05))
inlier_fraction_tol = float(rospy.get_param(
    param_name="/hdl_tf_node/inlier_fraction_tol", default=0.95))


hz = 100


class LowPassFilter:
    def __init__(self, cutoff_freq, ts):
        self.ts = ts
        self.cutoff_freq = cutoff_freq
        self.pre_out = 0.
        self.tau = self.calc_filter_coef()

    def calc_filter_coef(self):
        w_cut = 2 * np.pi * self.cutoff_freq
        return 1 / w_cut

    def filter(self, data):
        out = (self.tau * self.pre_out + self.ts * data) / (self.tau + self.ts)
        self.pre_out = out
        return out


class AverageFilter:
    def __init__(self):
        self.n = 0
        self.prev = 0.0

    def filter(self, data):
        self.n += 1
        alpha = (self.n - 1) / (self.n + 0.0)
        ave = alpha * self.prev + (1 - alpha) * data
        self.prev = ave
        return ave


class DataQueue(Queue):
    def __init__(self, length=10, init=True):
        super(DataQueue, self).__init__(length, init)
        self.__array = [init for i in range(length)]

    def inputValue(self, value):
        self.__array.append(value)
        del self.__array[0]

    def getAverage(self):
        return np.mean(np.array(self.__array))


class HDL_tf(object):
    def __init__(self):
        self.hdl_tf_sub = rospy.Subscriber(
            "/hdl_tf", HDL_TF, callback=self.tfCallback)
        self.status_sub = rospy.Subscriber(
            "/status", ScanMatchingStatus, callback=self.statusCallback)
        self.odom_sub = rospy.Subscriber(
            "/odometry/global", Odometry, callback=self.odomCallback
        )

        self.q_x = DataQueue(init=0., length=10)
        self.q_y = DataQueue(init=0., length=10)
        self.q_z = DataQueue(init=0., length=10)
        self.q_yaw = DataQueue(init=0., length=10)

        self.init_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.odom = Odometry()

        self.matching_err_queue = Queue(length=10, init=False)

        self.matching_error = float("inf")
        self.inlier_fraction = 0.

        self.trans = None
        self.rot = None

    # HDL_TF to ros tf

    def tfCallback(self, msg):
        assert type(msg) == type(HDL_TF())

        x, y, z = self.translationToArray(msg.translation)
        quat = self.rotationToArray(msg.rotation)
        _, _, yaw = euler_from_quaternion(quat)

        self.q_x.inputValue(x)
        self.q_y.inputValue(y)
        self.q_z.inputValue(z)
        self.q_yaw.inputValue(yaw)

        if self.matching_err_queue.isTrue(threshhold=10):
            self.trans = [self.q_x.getAverage(), self.q_y.getAverage(),
                          self.q_z.getAverage()]
            self.rot = quaternion_from_euler(
                0., 0., self.q_yaw.getAverage())

    # Status

    def statusCallback(self, msg):
        self.matching_error = msg.matching_error
        self.inlier_fraction = msg.inlier_fraction

        self.matching_err_queue.inputValue(
            self.matching_error <= matching_err_tol and self.inlier_fraction >= inlier_fraction_tol)

    # Odom
    def odomCallback(self, msg):
        self.odom = msg

    # broadcast TF from self.trans and rot
    def broadcastTF(self):
        if self.canTransform():
            self.tf_broadcaster.sendTransform(
                translation=self.trans,
                rotation=self.rot,
                time=rospy.Time.now(),
                child="odom",
                parent="map"
            )

    # Utils

    def translationToArray(self, trans):
        return [trans.x, trans.y, trans.z]

    def rotationToArray(self, rot):
        return [rot.x, rot.y, rot.z, rot.w]

    def transformOdometryToPose(self, odometry):
        pose = PoseStamped()

        pose.header.frame_id = odometry.header.frame_id
        pose.header.stamp = rospy.Time(0)
        pose.pose = odometry.pose.pose

        return pose

    def transformPoseToPoseWithCov(self, pose):
        poseCov = PoseWithCovarianceStamped()

        poseCov.header = pose.header
        poseCov.pose.pose = pose.pose

        return poseCov

    def canTransform(self):
        return self.trans is not None and self.rot is not None


if __name__ == "__main__":
    rospy.init_node("hdl_tf_node")

    hdl = HDL_tf()

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        hdl.broadcastTF()
        r.sleep()
