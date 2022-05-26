#!/usr/bin/env python

import math
import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class State(object):
    def __init__(self, odometry_topic="/odometry/global"):

        # Subscriber
        self.odom_sub = rospy.Subscriber(
            odometry_topic, Odometry, callback=self.odomCallback())

        self.tf_sub = tf.TransformListener()

        # Custum Field
        self.data = Odometry()
        self.yaw = 0.
        self.v = 0.

        # Calculation
        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

    def odomCallback(self, msg):
        self.currentTime = rospy.Time.now()

        msg = self.transformFrame(data=msg, target_frame="map")

        dx = msg.pose.pose.position.x - self.data.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.data.pose.pose.position.y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        dt = (self.lastTime - self.currentTime).to_sec()

        self.v = distance / dt

        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, self.yaw = euler_from_quaternion(quat)

        self.lastTime = self.currentTime
        self.data = msg

    def transformOdometryToPoseStamped(self, odom):
        pose = PoseStamped()

        pose.header.frame_id = odom.header.frame_id
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

        if self.tf_sub.canTransform(target_frame, source_frame, rospy.Time(0)):
            pose = self.tf_sub.transformPose(ps=self.transformOdometryToPoseStamped(
                self.data), target_frame=target_frame)
            odom = self.transformPoseStampedToOdometry(pose)
            return odom

        else:
            rospy.logwarn("Cannot Lookup Transform Between " +
                          target_frame + " and " + source_frame)
            return None
