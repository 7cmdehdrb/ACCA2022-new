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


score_threshold = rospy.get_param("/global_localizer/score_threshold", 1.0)
<<<<<<< HEAD




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

=======
>>>>>>> 49c6ee39461ef4be9f8b288f801210a05507a7a2


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
    rospy.init_node("global_localizer")\

    hz = 10
    freq = 1 / hz

    hz = 100.
    freq = 1 / hz

    map_frame = Odom(topic="/ndt_matching/ndt_pose")
    odom_frame = Odom(topic="/odometry/kalman")
    scan_status = ScanStatus()

    initial_pose_pub = rospy.Publisher(
        "/initialpose", PoseWithCovarianceStamped, queue_size=1)

    tf_pub = tf.TransformBroadcaster()
    tf_sub = tf.TransformListener()

<<<<<<< HEAD
    dx = 0.
    dy = 0.

    tx = 0.
    ty = 0.
    tyaw = 0.

    ave_x = LowPassFilter(cutoff_freq=5.0, ts=freq)
    ave_y = LowPassFilter(cutoff_freq=5.0, ts=freq)
    ave_yaw = LowPassFilter(cutoff_freq=5.0, ts=freq)


    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        if scan_status.queue.isTrue(threshhold=10):
            if tf_sub.canTransform("map", "odom", rospy.Time(0)):
                if odom_frame.frame_id is not None and map_frame.frame_id is not None:
                    p2 = tf_sub.transformPose("map", odom_frame.pose)

                    dx = map_frame.pose.pose.position.x - p2.pose.position.x
                    dy = map_frame.pose.pose.position.y - p2.pose.position.y
                    _, _, p2_yaw = euler_from_quaternion([p2.pose.orientation.x, p2.pose.orientation.y, p2.pose.orientation.z, p2.pose.orientation.w])
                    dyaw = map_frame.yaw - p2_yaw

                    tx += dx
                    ty += dy
                    tyaw += dyaw
                    
                    # print(dx, dy, dyaw)

                    rospy.loginfo(
                        "Translation Matrix : [%.6f, %.6f, 0.0]" % (tx, ty))
                    rospy.loginfo("Rotation Matrix : [0.0, 0.0, %.6f]" % tyaw)

                else:
                    rospy.logwarn("Cannot Subscribe odom topics")
=======
    trans = (0., 0., 0.)
    rot = (0., 0., 0., 1.)

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        if odom_frame.frame_id is not None and map_frame.frame_id is not None:
            # Subscribe odom / map
            if scan_status.queue.isTrue(threshhold=10):
                # Trustable TF
                dyaw = map_frame.yaw - odom_frame.yaw

                trans = (
                    map_frame.pose.pose.position.x -
                    (odom_frame.pose.pose.position.x * m.cos(dyaw) -
                     odom_frame.pose.pose.position.y * m.sin(dyaw)),
                    map_frame.pose.pose.position.y -
                    (odom_frame.pose.pose.position.x * m.sin(dyaw) +
                     odom_frame.pose.pose.position.y * m.cos(dyaw)),
                    0.
                )
                rot = quaternion_from_euler(0., 0., dyaw)

            elif scan_status.queue.isFalse(threshhold=10):
                # Untrustable TF : Relocalize
                if tf_sub.canTransform("map", "odom", rospy.Time(0)):
                    transformed_pose = tf_sub.transformPose(
                        ps=odom_frame.pose, target_frame="map")

                    p = PoseWithCovarianceStamped(
                        Header(None, rospy.Time.now(), "map"),
                        PoseWithCovariance(
                            transformed_pose.pose,
                            None
                        )
                    )

                    initial_pose_pub.publish(p)

                    scan_status.queue.inputValue(True)

                    rospy.logwarn("Relocalizing...")

                else:
                    # Cannot Transform
                    rospy.logwarn(
                        "Cannot lookup transform between map and odom")
>>>>>>> 49c6ee39461ef4be9f8b288f801210a05507a7a2

            else:
                rospy.logwarn("Scan is not trustable : %.4f" %
                              scan_status.scroe)

        else:
            # Cannot subscribe map / odom
            rospy.logwarn("Wait for Odometry...")

        tf_pub.sendTransform(
            translation=trans,
            rotation=rot,
            time=rospy.Time.now(),
            child="odom",
            parent="map"
        )

        r.sleep()
