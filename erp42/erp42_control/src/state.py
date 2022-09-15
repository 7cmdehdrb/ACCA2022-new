#!/usr/bin/env python

import math
import rospy
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from erp42_msgs.msg import SerialFeedBack


class State(object):
    def __init__(self, odometry_topic="/ndt_matching/ndt_pose", hz=30, test=True):

        # Subscriber
        self.odom_sub = rospy.Subscriber(
            odometry_topic, Odometry, callback=self.odomCallback)

        # Custum Field
        self.hz = hz
        self.data = Odometry()
        self.test = test

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
        dt = (self.currentTime - self.lastTime).to_sec()

        if 1 / dt > self.hz * 2.0:
            return 0
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quat)

        self.x = msg.pose.pose.position.x - \
            (0.0 if self.test is True else (1.040 / 2) * math.cos(yaw))
        self.y = msg.pose.pose.position.y - \
            (0.0 if self.test is True else (1.040 / 2) * math.sin(yaw))

        dx = msg.pose.pose.position.x - self.data.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.data.pose.pose.position.y

        distance = math.sqrt(dx ** 2 + dy ** 2)

        self.v = distance / dt

        self.omega = (yaw - self.yaw) / dt
        self.yaw = yaw

        self.data = msg

        self.lastTime = self.currentTime

    def getArray(self):
        # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        return np.array([self.x, self.y, self.yaw, self.v, self.omega])


class OdomState(State):
    def __init__(self, odometry_topic="/odometry/kalman", hz=30):
        super(OdomState, self).__init__(odometry_topic, hz)
        self.tf_sub = tf.TransformListener()

    def odomCallback(self, msg):
        self.currentTime = rospy.Time.now()
        dt = (self.currentTime - self.lastTime).to_sec()

        if (1 / dt) > self.hz * 2.0:
            return 0

        msg = self.transformFrame(data=msg, target_frame="map")

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        dx = msg.pose.pose.position.x - self.data.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.data.pose.pose.position.y

        distance = math.sqrt(dx ** 2 + dy ** 2)

        self.v = distance / dt

        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quat)

        self.omega = (yaw - self.yaw) / dt
        self.yaw = yaw

        self.data = msg

        self.lastTime = self.currentTime

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
    rospy.init_node("state")

    state1 = State(odometry_topic="/ndt_matching/ndt_pose", hz=10)
    state2 = OdomState(odometry_topic="/odometry/kalman", hz=30)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("MAP:\n\tX : %.6f\n\tY : %.6f\n\tV : %.6f" %
              (state1.x, state1.y, state1.v))
        print("ODOM:\n\tX : %.6f\n\tY : %.6f\n\tV : %.6f" %
              (state2.x, state2.y, state2.v))
        print("DELTA:\n\tX : %.6f\n\tY : %.6f\n\tV : %.6f" %
              (state2.x - state1.x, state2.y - state1.y, state2.v - state1.v))
        print("")
        r.sleep()
