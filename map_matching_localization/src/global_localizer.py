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
from header import Queue


score_threshold = rospy.get_param("/global_localizer/score_threshold", 0.5)


class Odom(object):
    def __init__(self, topic):
        self.topic = topic
        self.frame_id = None
        self.child_frame_id = None

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
        self.sub = rospy.Subscriber(
            "/ndt_matching/ndt_stat", NDTStat, callback=self.statCallback)

        self.scroe = float("inf")
        self.queue = Queue(length=10, init=True)

    def statCallback(self, msg):
        self.scroe = msg.score
        self.queue.inputValue(msg.score < score_threshold)


if __name__ == "__main__":
    rospy.init_node("global_localizer")

    map_frame = Odom(topic="/ndt_matching/ndt_pose")
    odom_frame = Odom(topic="/odometry/kalman")
    scan_status = ScanStatus()

    tf_pub = tf.TransformBroadcaster()
    tf_sub = tf.TransformListener()

    tx = 0.
    ty = 0.
    tyaw = 0.

    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        if scan_status.queue.isTrue(threshhold=10):
            if tf_sub.canTransform("map", "odom", rospy.Time(0)):
                if odom_frame.frame_id is not None and map_frame.frame_id is not None:
                    p2 = tf_sub.transformPose("map", odom_frame.pose)

                    dx = map_frame.pose.pose.position.x - p2.pose.position.x
                    dy = map_frame.pose.pose.position.y - p2.pose.position.y

                    tx += dx
                    ty += dy

                    rospy.loginfo(
                        "Translation Matrix : [%.6f, %.6f, 0.0]" % (tx, ty))

            else:
                tx = map_frame.pose.pose.position.x - odom_frame.pose.pose.position.x
                ty = map_frame.pose.pose.position.y - odom_frame.pose.pose.position.y
                tyaw = map_frame.yaw - odom_frame.yaw

                rospy.logwarn("Wait for transform between map and odom")

        else:
            rospy.logwarn("Scan is not trustable : %.4f" % scan_status.scroe)

        tf_pub.sendTransform(
            translation=(tx, ty, 0.),
            rotation=quaternion_from_euler(0., 0., tyaw),
            time=rospy.Time.now(),
            child="odom",
            parent="map"
        )

        r.sleep()
