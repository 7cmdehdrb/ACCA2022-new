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
from autoware_msgs.msg import NDTStat


class Odom(object):
    def __init__(self, topic):
        self.topic = topic

        self.sub = rospy.Subscriber(
            self.topic, Odometry, callback=self.odomCallback)

        self.pose = PoseStamped()
        self.yaw = 0.

    def odomCallback(self, msg):
        self.frame_id = msg.header.frame_id
        self.child_frame_id = msg.child_frame_id

        p = PoseStamped()

        p.header.frame_id = msg.header.frame_id
        p.header.stamp = rospy.Time(0)

        p.pose = msg.pose.pose

        self.pose = p
        _, _, self.yaw = euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])


class ScanStatus(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/ndt_matching/ndt_stat")


def getTranslationAndRotation(odom_parent, odom_child):
    trans = (odom_parent.point.x - odom_child.point.x, odom_parent.point.y -
             odom_child.point.y, odom_parent.point.z - odom_child.point.x)
    rot = quaternion_from_euler(0, 0, odom_parent.yaw - odom_child.yaw)

    return trans, rot


if __name__ == "__main__":
    rospy.init_node("tf_test")

    map_frame = Odom(topic="/ndt_matching/ndt_pose")
    odom_frame = Odom(topic="/odometry/kalman")

    tf_pub = tf.TransformBroadcaster()
    tf_sub = tf.TransformListener()

    tx = 0.
    ty = 0.
    tyaw = 0.

    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        if tf_sub.canTransform("map", "odom", rospy.Time(0)):
            p2 = tf_sub.transformPose("map", odom_frame.pose)

            dx = map_frame.pose.pose.position.x - p2.pose.position.x
            dy = map_frame.pose.pose.position.y - p2.pose.position.y

            tx += dx
            ty += dy

            rospy.loginfo("Translation Matrix : [%.6f, %.6f, 0.0]" % (tx, ty))

        else:
            tx = map_frame.pose.pose.position.x - odom_frame.pose.pose.position.x
            ty = map_frame.pose.pose.position.y - odom_frame.pose.pose.position.y
            tyaw = map_frame.yaw - odom_frame.yaw

            rospy.logwarn("Wait for transform between map and odom")

        tf_pub.sendTransform(
            translation=(tx, ty, 0.),
            rotation=quaternion_from_euler(0., 0., tyaw),
            time=rospy.Time.now(),
            child="odom",
            parent="map"
        )

        r.sleep()


"""
if __name__ == "__main__":
    rospy.init_node("tf_test")

    pub1 = rospy.Publisher("p1", PoseStamped, queue_size=1)
    pub2 = rospy.Publisher("p2", PoseStamped, queue_size=1)
    tf_pub = tf.TransformBroadcaster()
    tf_sub = tf.TransformListener()

    tx = 0.
    ty = 0.

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

        # =======================

        rot = quat2

        if tf_sub.canTransform("map", "base_link", rospy.Time(0)):
            p1.header.stamp = rospy.Time(0)
            transformed_pose = tf_sub.transformPose("map", p1)

            x = transformed_pose.pose.position.x
            y = transformed_pose.pose.position.y

            dx = x2[0] - x
            dy = x2[1] - y

            tx += dx
            ty += dy

            print(dx, dy)

        else:
            tx = x2[0] - x1[0]
            ty = x2[1] - x2[0]
            rospy.logwarn("Cannot lookup transform")

        tf_pub.sendTransform(
            translation=(tx, ty, 0.),
            rotation=rot,
            time=rospy.Time.now(),
            child="base_link",
            parent="map",
        )

        r.sleep()
"""
