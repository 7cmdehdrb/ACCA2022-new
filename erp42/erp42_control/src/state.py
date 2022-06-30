#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class State(object):
    def __init__(self, odometry_topic="/odometry/global"):

        # Subscriber
        self.odom_sub = rospy.Subscriber(
            odometry_topic, Odometry, callback=self.odomCallback)

        self.tf_sub = tf.TransformListener()

        # Custum Field
        self.data = Odometry()

        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s
        self.omega = 0.  # rad/s

        # Calculation
        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

    def odomCallback(self, msg):
        self.currentTime = rospy.Time.now()

        msg = self.transformFrame(data=msg, target_frame="map")

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        dx = msg.pose.pose.position.x - self.data.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.data.pose.pose.position.y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        dt = (self.currentTime - self.lastTime).to_sec()

        self.v = distance / dt

        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quat)

        self.omega = (yaw - self.yaw) / dt
        self.yaw = yaw

        self.lastTime = self.currentTime

        self.data = msg

    def transformOdometryToPoseStamped(self, odom):
        pose = PoseStamped()

        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time(0)

        pose.pose = odom.pose.pose

        # print(odom)

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

    def getArray(self):
        # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        return np.array([self.x, self.y, self.yaw, self.v, self.omega])
