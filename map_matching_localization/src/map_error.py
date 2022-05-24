#!/usr/bin/env python

import rospy
import tf
import math as m
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from map_matching_localization.msg import Error


class Odom(object):
    def __init__(self, topic):
        assert type(topic) == str

        # Subscriber
        rospy.Subscriber(topic, Odometry, self.odomCallback)

        # Field
        self.topic = topic
        self.odom = Odometry()
        self.before = Point()
        self.current = Point()
        self.before_yaw = 0
        self.current_yaw = 0

    def odomCallback(self, msg):
        self.odom = msg

    def handleData(self):
        x = self.odom.pose.pose.orientation.x
        y = self.odom.pose.pose.orientation.y
        z = self.odom.pose.pose.orientation.z
        w = self.odom.pose.pose.orientation.w

        quat = [x, y, z, w]

        _, _, yaw = tf.transformations.euler_from_quaternion(quat)

        return yaw

    def calculatePosition(self):
        self.current = self.odom.pose.pose.position
        self.current_yaw = self.handleData()

        # logic
        distance = m.sqrt((self.current.x-self.before.x)**2 +
                          (self.current.y-self.before.y)**2)
        Yaw = self.current_yaw - self.before_yaw

        #print(pose_x, pose_y)
        self.before = self.current
        self.before_yaw = self.current_yaw

        return [distance, m.degrees(Yaw)]


class Hdl_Odometry(Odom):
    def __init__(self, topic):
        super(Hdl_Odometry, self).__init__(topic)

    def odomCallback(self, msg):
        self.odom = msg
        error.map_error()


class Odometry_Error(object):
    def __init__(self):
        # Object
        self.ekf_odom = Odom("/odometry/global")
        self.hdl_odom = Hdl_Odometry("/odom")
        self.distance = float("inf")
        self.yaw = m.pi

    def map_error(self):
        ekf_err = self.ekf_odom.calculatePosition()
        hdl_err = self.hdl_odom.calculatePosition()

        # logic hdl
        self.distance = abs(hdl_err[0]-ekf_err[0])
        self.yaw = abs(hdl_err[1]-ekf_err[1])

        return self.distance, self.yaw


if __name__ == "__main__":
    rospy.init_node('odometry_error')

    error = Odometry_Error()
    error_pub = rospy.Publisher("/error", Error, queue_size=1)
    error_data = Error()
    r = rospy.Rate(4.0)
    while not rospy.is_shutdown():
        error_data.distance = error.distance
        error_data.degree = error.yaw
        error_pub.publish(error_data)
        rospy.loginfo("%f, %f", error.distance, error.yaw)
        r.sleep()
