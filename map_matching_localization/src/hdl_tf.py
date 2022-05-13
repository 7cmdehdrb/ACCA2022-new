#!/usr/bin/env python


import rospy
import tf
from geometry_msgs.msg import PoseStamped


class HDL_tf(object):
    def __init__(self):
        self.sub = rospy.Subscriber(
            "/hdl_tf", PoseStamped, callback=self.tfCallback)

        self.tf_broadcaster = tf.TransformBroadcaster()

        self.child_frame = "odom"
        self.trans = [0., 0., 0.]
        self.rot = [0., 0., 0., 1.]

    def tfCallback(self, msg):
        # assert type(msg) == type(PoseStamped())

        self.trans = [msg.pose.position.x,
                      msg.pose.position.y, msg.pose.position.z]
        self.rot = [msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w]

    def broadcastTF(self):
        self.tf_broadcaster.sendTransform(
            translation=self.trans,
            rotation=self.rot,
            time=rospy.Time.now(),
            child=self.child_frame,
            parent="map"
        )


if __name__ == "__main__":
    rospy.init_node("hdl_tf_node")

    hdl = HDL_tf()

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        hdl.broadcastTF()
        r.sleep()
