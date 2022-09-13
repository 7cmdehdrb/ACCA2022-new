#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math as m
import numpy as np
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from erp42_control.msg import ControlMessage


MAX_STEER = rospy.get_param("/max_steer", 30.0)
L = rospy.get_param("base_length", 1.040)


def kph2mps(value):
    return value * 0.277778


def mps2kph(value):
    return value * 3.6


class FakeOdom(object):
    def __init__(self):
        self.__odom_pub = rospy.Publisher(
            "/odometry/kalman", Odometry, queue_size=1)
        self.cmd_sub = rospy.Subscriber(
            "/cmd_msg", ControlMessage, callback=self.cmdCallback)
        self.__tf_pub = tf.TransformBroadcaster()

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.__cmd_msg = ControlMessage()
        self.__odom = Odometry()
        # quat = quaternion_from_euler(0, 0, m.pi)
        # self.__odom.pose.pose.orientation = Quaternion(
        #     quat[0], quat[1], quat[2], quat[3])

        self.__odom.header.frame_id = "map"
        self.__odom.child_frame_id = "base_link"

    def cmdCallback(self, msg):
        self.__cmd_msg = msg

    def publish_TF(self):
        pose = self.__odom.pose.pose
        position = pose.position
        quat = pose.orientation

        self.__tf_pub.sendTransform(
            translation=(position.x, position.y, position.z),
            rotation=(quat.x, quat.y, quat.z, quat.w),
            time=self.current_time,
            parent="map",
            child="base_link"
        )

        return 0

    def publish_odom(self):
        self.__odom_pub.publish(self.__odom)
        return 0

    def update_odom(self):
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        vel = kph2mps(self.__cmd_msg.Speed) * \
            (1.0 if self.__cmd_msg.Gear == 2 else -1.0)
        steer = m.radians(self.__cmd_msg.Steer)

        quat = self.__odom.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        dx = vel * m.cos(yaw) * dt
        dy = vel * m.sin(yaw) * dt

        yaw += (vel / L) * m.tan(-steer) * dt
        quat = quaternion_from_euler(roll, pitch, yaw)

        self.__odom.header.stamp = self.current_time

        self.__odom.pose.pose.position.x += dx
        self.__odom.pose.pose.position.y += dy

        self.__odom.pose.pose.orientation.x = quat[0]
        self.__odom.pose.pose.orientation.y = quat[1]
        self.__odom.pose.pose.orientation.z = quat[2]
        self.__odom.pose.pose.orientation.w = quat[3]

        self.last_time = self.current_time

        return 0

    def loop(self):
        self.update_odom()
        self.publish_odom()
        self.publish_TF()

    def get_odom(self):
        return self.__odom


if __name__ == "__main__":
    rospy.init_node("fake_odom")

    fake_odom = FakeOdom()

    r = rospy.Rate(30.)
    while not rospy.is_shutdown():
        fake_odom.loop()
        r.sleep()
