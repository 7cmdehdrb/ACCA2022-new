#!/usr/bin/env python

import rospy
import tf
import threading
from geometry_msgs.msg import PoseWithCovarianceStamped
from hdl_localization.msg import HDL_TF
from header import *


"""

    TO DO: Write Detail

"""


# Param

hdl_topic = rospy.get_param(
    param_name="/hdl_tf_node/hdl_topic", default="hdl_tf")


class HDL_tf(object):
    def __init__(self, hz=1):
        self.__hdl_tf_sub = rospy.Subscriber(
            hdl_topic, HDL_TF, callback=self.tfCallback)
        self.__init_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)

        # Object
        self.__matching_err_queue = Queue()
        self.__hdl_state = HDL_State()
        self.__odom = OdometryGlobal()

        # TF
        self.__tf_broadcaster = tf.TransformBroadcaster()
        self.__tf_listener = tf.TransformListener()

        # Field
        self.__trans = None
        self.__rot = None
        self.__hz = hz
        self.__isValidTF = False

        # Threading
        th = threading.Thread(target=self.loop)
        th.start()

    # HDL_TF to ros tf
    def tfCallback(self, msg):

        assert type(msg) == type(HDL_TF())

        self.__matching_err_queue.inputValue(self.__hdl_state.isTrustable())

        # Only valid TF matrix every datas in queue is true
        if self.__matching_err_queue.isTrue(threshhold=10):
            self.__trans = translationToArray(msg.translation)
            self.__rot = rotationToArray(msg.rotation)
            self.__isValidTF = False

        # Only valid TF matrix every datas in queue is False
        elif self.__matching_err_queue.isFalse(threshhold=10):
            if self.isMatrixNotNone():
                self.__isValidTF = True

    def broadcastTF(self):
        if self.isMatrixNotNone():
            self.__tf_broadcaster.sendTransform(
                translation=self.__trans,
                rotation=self.__rot,
                time=rospy.Time.now(),
                child="odom",
                parent="map"
            )

    # do relocalizing via ekf
    def relocalize(self):
        if self.__tf_listener.canTransform("map", "odom", rospy.Time(0)):
            try:
                map_pose = self.__tf_listener.transformPose(
                    ps=self.__odom.transformOdometryToPose(), target_frame="map")
                pose_cov = transformPoseToPoseWithCov(map_pose)

                self.__init_pub.publish(pose_cov)

            except Exception as ex:
                rospy.logwarn(ex)

        else:
            rospy.logwarn("Cannot Transform Between Map and Odom")

    def isMatrixNotNone(self):
        return self.__trans is not None and self.__rot is not None

    def loop(self):
        r = rospy.Rate(self.__hz)
        while not rospy.is_shutdown():
            if self.__isValidTF is True:
                self.relocalize()
                rospy.logwarn("Invalid TF Relation... Trying Relocalization")
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("hdl_tf_node")

    hdl = HDL_tf(hz=1)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        hdl.broadcastTF()
        r.sleep()
