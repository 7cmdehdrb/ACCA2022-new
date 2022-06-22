#!/usr/bin/env python


import rospy
import tf
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from hdl_localization.msg import ScanMatchingStatus, HDL_TF


# Param

hdl_topic = rospy.get_param(
    param_name="/hdl_tf_node/hdl_topic", default="hdl_tf")
odom_topic = rospy.get_param(
    param_name="/hdl_tf_node/odom_topic", default="odometry/global")
matching_err_tol = float(rospy.get_param(
    param_name="/hdl_tf_node/matching_err_tol", default=0.05))
inlier_fraction_tol = float(rospy.get_param(
    param_name="/hdl_tf_node/inlier_fraction_tol", default=0.95))


class Queue(object):
    def __init__(self, length=10):
        self.__array = [True for i in range(length)]

    def inputValue(self, value):
        assert type(value) == bool

        self.__array.append(value)
        del self.__array[0]

    def count(self, flag):
        return self.__array.count(flag)

    def isTrue(self, threshhold=10):
        return self.count(True) >= threshhold

    def isFalse(self, threshhold=10):
        return self.count(False) >= threshhold


class HDL_tf(object):
    def __init__(self):
        self.hdl_tf_sub = rospy.Subscriber(
            "/hdl_tf", HDL_TF, callback=self.tfCallback)
        self.status_sub = rospy.Subscriber(
            "/status", ScanMatchingStatus, callback=self.statusCallback)
        self.odom_sub = rospy.Subscriber(
            "/odometry/global", Odometry, callback=self.odomCallback
        )

        self.init_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.odom = Odometry()

        self.matching_err_queue = Queue()

        self.matching_error = float("inf")
        self.inlier_fraction = 0.

        self.temp = False

        self.trans = None
        self.rot = None

        th = threading.Thread(target=self.loop)
        th.start()

    # HDL_TF to ros tf
    def tfCallback(self, msg):
        assert type(msg) == type(HDL_TF())

        # self.matching_err_queue.inputValue(
        #     self.matching_error <= 0.05 and self.inlier_fraction >= 0.95)

        self.trans = self.translationToArray(msg.translation)
        self.rot = self.rotationToArray(msg.rotation)
        
        # self.temp = True
        
        # if self.matching_err_queue.isTrue(threshhold=10):
        #     self.temp = False

        # elif self.matching_err_queue.isFalse(threshhold=10):
        #     if self.canTransform():
        #         self.temp = True
        #         # self.relocalize()
        #         # rospy.logwarn("Invalid TF Relation... Trying Relocalization")

    # Status
    def statusCallback(self, msg):
        self.matching_error = msg.matching_error
        self.inlier_fraction = msg.inlier_fraction

    # Odom
    def odomCallback(self, msg):
        self.odom = msg

    # broadcast TF from self.trans and rot
    def broadcastTF(self):
        if self.canTransform():
            rospy.loginfo("HELLO")
            self.tf_broadcaster.sendTransform(
                translation=self.trans,
                rotation=self.rot,
                time=rospy.Time.now(),
                child="odom",
                parent="map"
            )

    # do relocalizing via ekf
    def relocalize(self):
        if self.tf_listener.canTransform("map", "odom", rospy.Time(0)):
            try:
                map_pose = self.tf_listener.transformPose(
                    ps=self.transformOdometryToPose(self.odom), target_frame="map")
                pose_cov = self.transformPoseToPoseWithCov(map_pose)

                self.init_pub.publish(pose_cov)

            except Exception as ex:
                rospy.logwarn(ex)

        else:
            rospy.logwarn("Cannot Transform Between Map and Odom")

    def loop(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.temp is True:
                self.relocalize()
                rospy.logwarn("Invalid TF Relation... Trying Relocalization")
            r.sleep()

    # Utils

    def translationToArray(self, trans):
        return [trans.x, trans.y, trans.z]

    def rotationToArray(self, rot):
        return [rot.x, rot.y, rot.z, rot.w]

    def transformOdometryToPose(self, odometry):
        pose = PoseStamped()

        pose.header.frame_id = odometry.header.frame_id
        pose.header.stamp = rospy.Time(0)
        pose.pose = odometry.pose.pose

        return pose

    def transformPoseToPoseWithCov(self, pose):
        poseCov = PoseWithCovarianceStamped()

        poseCov.header = pose.header
        poseCov.pose.pose = pose.pose

        return poseCov

    def canTransform(self):
        return self.trans is not None and self.rot is not None


if __name__ == "__main__":
    rospy.init_node("hdl_tf_node")

    hdl = HDL_tf()

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        hdl.broadcastTF()
        r.sleep()
