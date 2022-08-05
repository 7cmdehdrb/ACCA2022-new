#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import *
from std_msgs.msg import Header


if __name__ == "__main__":
    rospy.init_node("tf_test")

    pub1 = rospy.Publisher("p1", PoseStamped, queue_size=1)
    pub2 = rospy.Publisher("p2", PoseStamped, queue_size=1)
    tf_pub = tf.TransformBroadcaster()

    r = rospy.Rate(1.)
    while not rospy.is_shutdown():
        p1 = PoseStamped()

        quat1 = quaternion_from_euler(0., 0., 0.)
        p1.header = Header(0, rospy.Time.now(), "base_link")
        p1.pose.position = Point(1, 1, 0)
        p1.pose.orientation = Quaternion(
            quat1[0], quat1[1], quat1[2], quat1[3])

        pub1.publish(p1)

        A = np.array([
            [m.cos(m.radians(30.)), -m.sin(m.radians(30.)), 0, 3],
            [m.sin(m.radians(30.)), m.cos(m.radians(30.)), 0, 6],
            [0, 0, 1, 9],
            [0, 0, 0, 1]
        ])

        x1 = np.array([[1], [1], [0], [1]])
        x2 = np.dot(A, x1)

        print(x2)

        # =======================

        p2 = PoseStamped()

        p2.header = Header(0, rospy.Time.now(), "map")
        p2.pose.position = Point(x2[0], x2[1], x2[2])
        p2.pose.orientation = Quaternion(
            quat1[0], quat1[1], quat1[2], quat1[3])

        pub2.publish(p2)

        trans = (3, 6, 9)
        rot = quaternion_from_euler(0, 0, m.radians(30.))

        tf_pub.sendTransform(
            translation=trans,
            rotation=rot,
            time=rospy.Time.now(),
            child="base_link",
            parent="map",
        )

        r.sleep()
