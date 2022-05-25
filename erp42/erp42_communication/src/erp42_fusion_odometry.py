#!/usr/bin/env python

import sys
import rospy
import numpy as np
import tf
import math as m
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped
from erp42_msgs.msg import SerialFeedBack


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/enbang/ACCA-master")

try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils/")
    from state import State
except ImportError as ie:
    print("UTIL IMPORT ERROR")
    print(ie)
    sys.exit()


current_time = None
last_time = None


class FusionState(State):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, odom=None):
        super(FusionState, self).__init__(x=x, y=y, yaw=yaw, v=v)

        self.odometry = odom
        self.quat = [0.0, 0.0, 0.0, 1.0]

        self.init_yaw = 0.0

    def update(self):
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()

        vel, quat = self.odometry.handleData()
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)

        # yaw = yaw - self.init_yaw

        #print(yaw)

        self.v = vel
        self.dx = vel * m.cos(yaw)
        self.dy = vel * m.sin(yaw)
        self.dyaw = yaw - self.last_yaw

        self.x += self.dx * dt
        self.y += self.dy * dt
        self.yaw = yaw
        self.quat = quat

        self.last_x = self.x
        self.last_y = self.y
        self.last_yaw = self.yaw
        

        # print(self.x, self.y)

        self.odometry.last_time = rospy.Time.now()


class FusionOdometry(object):
    def __init__(self):

        # Subscriber

        rospy.Subscriber("/erp42_feedback", SerialFeedBack,
                         callback=self.encoderCallback)
        rospy.Subscriber("/imu/data", Imu, callback=self.imuCallback)

        # msg

        self.SerialFeedBack = SerialFeedBack()
        self.imuMsg = Imu()

        # param

        self.doTransform = True
    

        # Value

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        #self.state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

    def encoderCallback(self, msg):
        self.SerialFeedBack = msg

    def imuCallback(self, msg):
        self.imuMsg = msg

        if state.init_yaw == 0.0:

            x = self.imuMsg.orientation.x
            y = self.imuMsg.orientation.y
            z = self.imuMsg.orientation.z
            w = self.imuMsg.orientation.w

            _, _, YAW = tf.transformations.euler_from_quaternion([x, y, z, w])
            state.init_yaw = YAW

            print("INITIALIZE SUCCESS")
            print(YAW)

    def handleData(self):

        vel = self.SerialFeedBack.speed  # m/s

        x = self.imuMsg.orientation.x
        y = self.imuMsg.orientation.y
        z = self.imuMsg.orientation.z
        w = self.imuMsg.orientation.w

        quat = [x, y, z, w]

        return vel, quat


if __name__ == "__main__":
    rospy.init_node("erp42_fusion_odometry")

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    odometry = FusionOdometry()
    state = FusionState(x=0.0, y=0.0, yaw=0.0, v=0.0, odom=odometry)

    odom_pub = rospy.Publisher(
        "/erp42_fusion_odometry", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    odom = Odometry()

    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():

        state.update()

        odom.header.stamp = rospy.Time.now()

        odom.pose.pose = Pose(
            Point(state.x, state.y, 0.0), Quaternion(*state.quat)
        )

        odom.twist.twist = Twist(
            Vector3(state.dx, state.dt, 0.0),
            Vector3(0.0, 0.0, state.dyaw)
        )


        if odometry.doTransform is True:

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, state.yaw)

            odom_broadcaster.sendTransform(
                (state.x, state.y, 0.0),
                quat,
                rospy.Time.now(),
                "base_link",
                "odom"
            )

        odom_pub.publish(odom)

        last_time = rospy.Time.now()

        r.sleep()
