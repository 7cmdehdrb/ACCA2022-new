#!/usr/bin/env python


import sys
import os
import rospy
import rospkg
import numpy as np
import math as m
import tf
from tf.transformations import *
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
import threading
from gaussian import Gaussian, gaussianConvolution


try:
    sys.path.append(rospkg.RosPack().get_path("erp42_control") + "/src")
    from state import State
except Exception as ex:
    rospy.logfatal(ex)
    rospy.logfatal("Error at gaussian_cross")


class HDLState(State):
    def __init__(self, odometry_topic="/odometry/kalman"):
        super(HDLState, self).__init__(odometry_topic)

    def odomCallback(self, msg):
        self.data = msg


class OdomState(State):
    def __init__(self, odometry_topic="/odometry/kalman"):
        super(OdomState, self).__init__(odometry_topic)
        self.tf_sub = tf.TransformListener()

    def odomCallback(self, msg):
        cov = msg.pose.covariance
        msg = self.transformFrame(data=msg, target_frame="map")

        self.data = msg
        self.data.pose.covariance = cov

    def transformOdometryToPoseStamped(self, odom):
        pose = PoseStamped()

        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time(0)

        pose.pose = odom.pose.pose

        return pose

    def transformPoseStampedToOdometry(self, pose):
        odom = Odometry()

        odom.header.frame_id = pose.header.frame_id
        odom.header.stamp = rospy.Time(0)

        odom.pose.pose = pose.pose

        return odom

    def transformFrame(self, data, target_frame="map"):
        source_frame = data.header.frame_id

        try:
            if self.tf_sub.canTransform(target_frame=target_frame, source_frame=source_frame, time=rospy.Time(0)):
                pose = self.tf_sub.transformPose(
                    ps=self.transformOdometryToPoseStamped(data), target_frame=target_frame)
                odom = self.transformPoseStampedToOdometry(pose)
                return odom

            else:
                raise Exception()

        except Exception as ex:
            rospy.logwarn("Cannot Lookup Transform Between " +
                          target_frame + " and " + source_frame)
            rospy.logwarn(ex)
            return Odometry()


if __name__ == "__main__":
    rospy.init_node("gaussian_cross")

    odometry = OdomState(odometry_topic="/odometry/kalman")
    matching = HDLState(odometry_topic="/odom")

    odom_pub = rospy.Publisher("/odometry/gaussian", Odometry, queue_size=1)

    hz = 10

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        if odometry.data.pose.covariance[0] == 0. or matching.data.pose.covariance[0] == 0.:
            rospy.logwarn("Waiting for data...")
            print(odometry.data.pose.covariance[0],
                  matching.data.pose.covariance[0])
            r.sleep()
            continue

        x1 = Gaussian(None, odometry.data.pose.pose.position.x,
                      odometry.data.pose.covariance[0])
        x2 = Gaussian(None, matching.data.pose.pose.position.x,
                      matching.data.pose.covariance[0])

        y1 = Gaussian(None, odometry.data.pose.pose.position.y,
                      odometry.data.pose.covariance[0])
        y2 = Gaussian(None, matching.data.pose.pose.position.y,
                      matching.data.pose.covariance[0])

        _, _, yaw1 = euler_from_quaternion([odometry.data.pose.pose.orientation.x, odometry.data.pose.pose.orientation.y,
                                           odometry.data.pose.pose.orientation.z, odometry.data.pose.pose.orientation.w])
        yaw1 = Gaussian(None, yaw1, odometry.data.pose.covariance[0])

        _, _, yaw2 = euler_from_quaternion([matching.data.pose.pose.orientation.x, matching.data.pose.pose.orientation.y,
                                            matching.data.pose.pose.orientation.z, matching.data.pose.pose.orientation.w])
        yaw2 = Gaussian(None, yaw2, matching.data.pose.covariance[0] + 1.5)

        position = (gaussianConvolution(x1, x2), gaussianConvolution(y1, y2))
        yaw = gaussianConvolution(yaw1, yaw2)
        quat = quaternion_from_euler(0., 0., yaw.mean)

        msg = Odometry()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.child_frame_id = "odom"

        msg.pose.pose.position = Point(position[0].mean, position[1].mean, 0.)
        msg.pose.pose.orientation = Quaternion(
            quat[0], quat[1], quat[2], quat[3])

        msg.pose.covariance = [
            position[0].sigma, 0., 0., 0., 0., 0.,
            0., position[1].sigma, 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
        ]

        odom_pub.publish(msg)

        r.sleep()
