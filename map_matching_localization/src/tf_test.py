#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from tf.transformations import *
from tf.msg import tfMessage
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class Odom(object):
    def __init__(self, topic):
        self.topic = topic
        self.frame_id = None
        self.child_frame_id = None

        self.sub = rospy.Subscriber(
            self.topic, Odometry, callback=self.odomCallback)

        self.point = Point()
        self.yaw = 0.

    def odomCallback(self, msg):
        self.frame_id = msg.header.frame_id
        self.child_frame_id = msg.child_frame_id

        self.point = msg.pose.pose.position
        _, _, self.yaw = euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])


def getTransformMatrix(odom_parent, odom_child):
    A = np.array([
        [m.cos(m.radians(odom_parent.yaw - odom_child.yaw)), -
         m.sin(m.radians(odom_parent.yaw - odom_child.yaw)), 0, odom_parent.point.x - odom_child.point.x],
        [m.sin(m.radians(odom_parent.yaw - odom_child.yaw)),
         m.cos(m.radians(odom_parent.yaw - odom_child.yaw)), 0, odom_parent.point.y - odom_child.point.y],
        [0, 0, 1, odom_parent.point.z - odom_child.point.x],
        [0, 0, 0, 1]
    ])

    return A


def getTranslationAndRotation(odom_parent, odom_child):
    trans = (odom_parent.point.x - odom_child.point.x, odom_parent.point.y -
             odom_child.point.y, odom_parent.point.z - odom_child.point.x)
    rot = quaternion_from_euler(0, 0, odom_parent.yaw - odom_child.yaw)

    return trans, rot


if __name__ == "__main__":
    rospy.init_node("tf_test")

    # tf_broadcaster = tf.TransformBroadcaster()
    # map = Odom("/ndt_matching/ndt_pose")
    # odom = Odom("/odometry/kalman")

    # hz = 100

    # r = rospy.Rate(hz)
    # while not rospy.is_shutdown():

    #     trans, rot = getTranslationAndRotation(map, odom)

    #     tf_broadcaster.sendTransform(
    #         translation=trans,
    #         rotation=rot,
    #         time=rospy.Time.now(),
    #         child="odom",
    #         parent="map",
    #     )

    #     r.sleep()

    pub1 = rospy.Publisher("p1", PoseStamped, queue_size=1)
    pub2 = rospy.Publisher("p2", PoseStamped, queue_size=1)
    tf_pub = tf.TransformBroadcaster()

    r = rospy.Rate(1.)
    while not rospy.is_shutdown():

        x1 = np.array([1, 1, 0, 1])
        x2 = np.array([10, 15, 0, 1])
        deg = 60.

        p1 = PoseStamped()

        quat1 = quaternion_from_euler(0., 0., 0.)
        p1.header = Header(0, rospy.Time.now(), "base_link")
        p1.pose.position = Point(x1[0], x1[1], x1[2])
        p1.pose.orientation = Quaternion(
            quat1[0], quat1[1], quat1[2], quat1[3])

        pub1.publish(p1)

        # =======================

        quat2 = quaternion_from_euler(0., 0., m.radians(deg))

        p2 = PoseStamped()

        p2.header = Header(0, rospy.Time.now(), "map")
        p2.pose.position = Point(x2[0], x2[1], x2[2])
        p2.pose.orientation = Quaternion(
            quat2[0], quat2[1], quat2[2], quat2[3])

        pub2.publish(p2)

        tf_matrix = TransformStamped()

        tf_matrix.header.stamp = rospy.Time.now()
        tf_matrix.header.frame_id = "map"
        tf_matrix.child_frame_id = "base_link"

        # tf_pub.sendTransform(
        #     translation=trans,
        #     rotation=rot,
        #     time=rospy.Time.now(),
        #     child="base_link",
        #     parent="map",
        # )

        r.sleep()
