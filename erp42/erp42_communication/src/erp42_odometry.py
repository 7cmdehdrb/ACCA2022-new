#!/usr/bin/env python

import rospy
import tf
import math as m
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from erp42_msgs.msg import SerialFeedBack


class State():
    def __init__(self, odom=None):
        self.odometry = odom
        self.quat = [0.0, 0.0, 0.0, 1.0]

        self.init_yaw = 0.0

    def update(self):

        vel, quat = self.odometry.handleData()
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        print(yaw)
        # speed
        self.v = vel
        self.dx = vel * m.cos(yaw)
        self.dy = vel * m.sin(yaw)


class erp42_Odometry(object):
    def __init__(self):
        # Subsriber
        rospy.Subscriber("/erp42_feedback", SerialFeedBack,
                         callback=self.encoderCallback)
        rospy.Subscriber("/imu/data", Imu, callback=self.imuCallback)

        # msg
        self.SerialFeedBack = SerialFeedBack()
        self.imuMsg = Imu()
        self.current_time = 0
        self.before_time = 0
        # param
        self.doTransform = True

    def encoderCallback(self, msg):
        self.SerialFeedBack = msg

    def imuCallback(self, msg):
        self.imuMsg = msg

        if state.init_yaw == 0.0:
            x = self.imuMsg.orientation.x
            y = self.imuMsg.orientation.y
            z = self.imuMsg.orientation.z
            w = self.imuMsg.orientation.w
            _, _, Yaw = tf.transformations.euler_from_quaternion([x, y, z, w])
            state.init_yaw = Yaw

    def handleData(self):
        vel = self.SerialFeedBack.speed

        x = self.imuMsg.orientation.x
        y = self.imuMsg.orientation.y
        z = self.imuMsg.orientation.z
        w = self.imuMsg.orientation.w

        quat = [x, y, z, w]

        return vel, quat


if __name__ == "__main__":
    rospy.init_node('erp42_odometry')

    odometry = erp42_Odometry()
    state = State(odom=odometry)

    odom_pub = rospy.Publisher("/wheel/odometry", Odometry, queue_size=2)

    odom = Odometry()

    odom.header.frame_id = "encoder"
    odom.child_frame_id = "base_link"

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():

        state.update()
        odom.twist.twist = Twist(
            Vector3(state.dx, state.dy, 0.0), Vector3(0.0, 0.0, 0.0))

        odom_pub.publish(odom)

        r.sleep()
