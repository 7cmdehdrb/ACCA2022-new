#!/usr/bin/env python

import rospy
import numpy as np
import math as m
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from erp42_msgs.msg import SerialFeedBack
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import random


def kph2mps(value):
    return value * 0.277778


def mps2kph(value):
    return value * 3.6


class Kalman(object):
    def __init__(self):
        # initial value
        self.x = np.array([
            0, 0, 0, 0
        ])
        self.P = np.array([
            [0.5, 0, 0, 0],
            [0, 0.5, 0., 0.],
            [0, 0., 0.5, 0.],
            [0, 0., 0., 0.5]
        ])

        # noise
        self.Q = np.array([
            [0.5, 0, 0, 0],
            [0, 0.5, 0., 0.],
            [0, 0., 0.5, 0.],
            [0, 0., 0., 0.5]
        ])

        self.R1 = np.array([
            [5., 0., 0., 0.],
            [0., 5., 0., 0.],
            [0., 0., 10., 0.],
            [0., 0., 0., 20.]
        ])
        self.R2 = np.array([
            [5000., 0., 0., 0.],
            [0., 5000., 0., 0.],
            [0., 0., 10., 0.],
            [0., 0., 0., 20.]
        ])

        self.dt = 0.

    def inv(self, matrix):
        return np.linalg.inv(matrix)

    def filter(self, z_gps, z_hdl, z_erp, z_imu, dt):
        A = np.array([
            [1, 0, m.cos(self.x[3]) * dt, 0],
            [0, 1, m.sin(self.x[3]) * dt, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        x_k = np.dot(A, self.x)
        P_k = np.dot(np.dot(A, self.P), A.T) + self.Q

        H_gps = H_hdl = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        H_erp = np.array([[0, 0, 0, 0], [0, 0, 0, 0],
                         [0, 0, 1, 0], [0, 0, 0, 0]])
        H_imu = np.array([[0, 0, 0, 0], [0, 0, 0, 0],
                         [0, 0, 0, 0], [0, 0, 0, 1]])

        K_gps = np.dot(np.dot(P_k, H_gps.T),
                       self.inv(np.dot(np.dot(H_gps, P_k), H_gps.T) + self.R1))
        K_hdl = np.dot(np.dot(P_k, H_hdl.T),
                       self.inv(np.dot(np.dot(H_hdl, P_k), H_hdl.T) + self.R2))
        K_erp = np.dot(np.dot(P_k, H_erp.T),
                       self.inv(np.dot(np.dot(H_erp, P_k), H_erp.T) + self.R1))
        K_imu = np.dot(np.dot(P_k, H_imu.T),
                       self.inv(np.dot(np.dot(H_imu, P_k), H_imu.T) + self.R1))

        X = x_k + np.dot(K_gps, (z_gps - np.dot(H_gps, x_k))) + np.dot(K_hdl, (z_hdl - np.dot(H_hdl, x_k))) + \
            np.dot(K_erp, (z_erp - np.dot(H_erp, x_k))) + \
            np.dot(K_imu, (z_imu - np.dot(H_imu, x_k)))

        self.x = X

        # self.P = P_k - K_gps * H_gps * P_k
        self.P = P_k - \
            np.dot(np.dot((K_gps + K_erp + K_imu), np.identity(n=4)), P_k)

        self.dt = dt

        return self.x


class Sensor(object):
    def __init__(self, topic, msg_type):
        super(Sensor, self).__init__()
        self.data = np.array([0, 0, 0, 0])
        self.sub = rospy.Subscriber(
            topic, msg_type, callback=self.sensorCallback)

    def sensorCallback(self, msg):
        self.data = self.handleData(msg)

    def handleData(self, msg):
        return np.array([0, 0, 0, 0])


class ERP42(Sensor):
    def __init__(self, topic, msg_type):
        super(ERP42, self).__init__(topic, msg_type)

    def handleData(self, msg):
        return np.array([0, 0, msg.speed, 0])


class Xsens(Sensor):
    def __init__(self, topic, msg_type):
        super(Xsens, self).__init__(topic, msg_type)

        self.yaw = 0.

        self.current = rospy.Time.now()
        self.last = rospy.Time.now()

    # def handleData(self, msg):
    #     quat = msg.orientation
    #     _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    #     return np.array([0, 0, 0, yaw])

    def handleData(self, msg):
        self.current = rospy.Time.now()

        self.yaw += (kph2mps(msg.speed) / 1.040) * \
            m.tan(-msg.steer) * kf.dt

        self.last = self.current

        return np.array([0, 0, 0, self.yaw])


class Odom(Sensor):
    def __init__(self, topic, msg_type):
        super(Odom, self).__init__(topic, msg_type)

    def handleData(self, msg):
        pose = msg.pose.pose.position

        return np.array([pose.x, pose.y, 0, 0])


if __name__ == "__main__":
    rospy.init_node("kalman")

    pub = rospy.Publisher("test", PoseStamped, queue_size=1)

    kf = Kalman()

    gps = Odom("/odometry/gps", Odometry)
    hdl = Odom("/odometry/gps", Odometry)
    erp = ERP42("/erp42_feedback", SerialFeedBack)
    imu = Xsens("/erp42_feedback", SerialFeedBack)

    hz = 10

    dt = 0.1

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        res = kf.filter(gps.data, hdl.data, erp.data, imu.data,
                        dt=(current_time - last_time).to_sec())

        print(res)

        msg = PoseStamped()

        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = res[0]
        msg.pose.position.y = res[1]

        quat = quaternion_from_euler(0, 0, res[3])

        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        pub.publish(msg)

        last_time = current_time

        r.sleep()
