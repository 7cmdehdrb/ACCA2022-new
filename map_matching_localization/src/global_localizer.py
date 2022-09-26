#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from tf.transformations import *
from tf.msg import tfMessage
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float32
from autoware_msgs.msg import NDTStat
from header import Queue
from gaussian import Gaussian, gaussianConvolution


<<<<<<< HEAD
score_threshold = rospy.get_param("/global_localizer/score_threshold", 5.0)


def thresholdCallback(msg):
    global score_threshold
    score_threshold = msg.data
=======
score_threshold = rospy.get_param("/global_localizer/score_threshold", 0.5)
>>>>>>> 8d41b4996f5ba5a91c0a109914e47946ebee631a


class Odom(object):
    def __init__(self, topic):
        self.topic = topic
        self.frame_id = None
        self.child_frame_id = None

        self.sub = rospy.Subscriber(
            self.topic, Odometry, callback=self.odomCallback)

        self.pose = PoseStamped()
        self.cov = float("inf")
        self.yaw = 0.

    def odomCallback(self, msg):
        self.frame_id = msg.header.frame_id
        self.child_frame_id = msg.child_frame_id

        self.cov = msg.pose.covariance[0]

        p = PoseStamped()

        p.header.frame_id = msg.header.frame_id
        p.header.stamp = rospy.Time(0)

        p.pose = msg.pose.pose

        self.pose = p
        _, _, self.yaw = euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])


class GPS(object):
    def __init__(self, topic="/ublox_gps/odometry"):
        self.sub = rospy.Subscriber(
            topic, PointStamped, callback=self.callback
        )

        self.point = None

    def callback(self, msg):
        self.point = msg.point


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

<<<<<<< HEAD
    hz = 30
    freq = 1 / hz

    # hz = 100.
    # freq = 1 / hz
=======
    ignore = True

    hz = 30.
    freq = 1 / hz
>>>>>>> 8d41b4996f5ba5a91c0a109914e47946ebee631a

    map_frame = Odom(topic="/ndt_matching/ndt_pose")
    odom_frame = Odom(topic="/odometry/kalman")
    gps = GPS(topic="/gps_tracker/odometry")
    scan_status = ScanStatus()

    initial_pose_pub = rospy.Publisher(
        "/initialpose", PoseWithCovarianceStamped, queue_size=1)

    tol_sub = rospy.Subscriber(
        "/global_localizer/threshold", Float32, callback=thresholdCallback
    )


    tf_pub = tf.TransformBroadcaster()
    tf_sub = tf.TransformListener()

    trans = (0., 0., 0.)
    rot = (0., 0., 0., 1.)
    dist = float("inf")

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        if odom_frame.frame_id is not None and map_frame.frame_id is not None and gps.point is not None:
            # Subscribe odom / map
            if scan_status.queue.isTrue(threshhold=10):
                # Trustable TF
<<<<<<< HEAD
                dyaw = map_frame.yaw - odom_frame.yaw

                new_trans = (
                    map_frame.pose.pose.position.x -
                    (odom_frame.pose.pose.position.x * m.cos(dyaw) -
                     odom_frame.pose.pose.position.y * m.sin(dyaw)),
                    map_frame.pose.pose.position.y -
                    (odom_frame.pose.pose.position.x * m.sin(dyaw) +
                     odom_frame.pose.pose.position.y * m.cos(dyaw)),
                    0.
                )
                new_rot = quaternion_from_euler(0., 0., dyaw)
                d = np.hypot(new_trans[0] - trans[0], new_trans[1] - trans[0])
                

                print(dist)

                if abs(dist - d) <= 1. or trans == (0., 0., 0.):
                    trans = new_trans
                    rot = new_rot

                dist = d
                
=======
                dist = np.hypot(map_frame.pose.pose.position.x - gps.point.x,
                                map_frame.pose.pose.position.y - gps.point.y)

                if dist < 2.0 or ignore is True:
                    # Distance between ndt matching and gps tracker
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

                else:
                    rospy.logwarn(
                        "Scan Status is good but there is distance gap between ndt mathcing and gps tracker...")

                    for _ in range(10):
                        scan_status.queue.inputValue(False)
>>>>>>> 8d41b4996f5ba5a91c0a109914e47946ebee631a

            elif scan_status.queue.isFalse(threshhold=10):
                # Untrustable TF : Relocalize
                if tf_sub.canTransform("map", "odom", rospy.Time(0)):
                    transformed_pose = tf_sub.transformPose(
                        ps=odom_frame.pose, target_frame="map")

                    ndt_x = Gaussian(
                        None, transformed_pose.pose.position.x, odom_frame.cov * 2.0)
                    ndt_y = Gaussian(
                        None, transformed_pose.pose.position.y, odom_frame.cov * 2.0
                    )

                    gps_x = Gaussian(
                        None, gps.point.x, 3.0
                    )
                    gps_y = Gaussian(
                        None, gps.point.y, 3.0
                    )

                    convolution_x = gaussianConvolution(ndt_x, gps_x)
                    convolution_y = gaussianConvolution(ndt_y, gps_y)

                    p = PoseWithCovarianceStamped(
                        Header(None, rospy.Time.now(), "map"),
                        PoseWithCovariance(
                            Pose(
                                Point(convolution_x.mean,
                                      convolution_y.mean, 0.0),
                                transformed_pose.pose.orientation
                            ),
                            None
                        )
                    )

                    initial_pose_pub.publish(p)

                    scan_status.queue.inputValue(True)

                    rospy.loginfo("Relocalizing...")

                else:
                    # Cannot Transform
                    rospy.logwarn(
                        "Cannot lookup transform between map and odom")

            else:
                pass
                # rospy.logwarn("Scan is not trustable : %.4f" %
                #               scan_status.scroe)

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

        print(trans)

        r.sleep()
