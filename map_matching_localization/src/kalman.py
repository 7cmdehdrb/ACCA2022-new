#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import random
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from erp42_msgs.msg import SerialFeedBack
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gaussian import *


def kph2mps(value):
    return value * 0.277778


def mps2kph(value):
    return value * 3.6


def getEmpty(shape):
    return np.array([[0. for i in range(shape[0])] for j in range(shape[1])])


class Kalman(object):
    def __init__(self, *args, **kwargs):
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

        self.dt = 0.

    def inv(self, matrix):
        return np.linalg.inv(matrix)

    def filter(self, gps, hdl, erp, imu, dt):
        A = np.array([
            [1, 0, m.cos(self.x[3]) * dt, 0],
            [0, 1, m.sin(self.x[3]) * dt, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        z_gps = [Gaussian(None, gps.data[0], gps.cov[0][0]),
                 Gaussian(None, gps.data[1], gps.cov[1][1])]    # [x, y]
        z_hdl = [Gaussian(None, hdl.data[0], hdl.cov[0][0]),
                 Gaussian(None, hdl.data[1], hdl.cov[1][1])]    # [x, y]

        position = [gaussianConvolution(
            z_gps[0], z_hdl[0]), gaussianConvolution(z_gps[1], z_hdl[1])]   # [gaussian x, gaussian y]

        z_position = np.array([position[0].mean, position[1].mean, 0., 0.])
        z_erp = erp.data
        z_imu = imu.data

        cov_position = getEmpty(shape=(4, 4))
        cov_erp = erp.cov
        cov_imu = imu.cov

        for i in range(2):
            cov_position[i][i] = position[i].sigma

        R = cov_position + cov_erp + cov_imu

        x_k = np.dot(A, self.x)
        P_k = np.dot(np.dot(A, self.P), A.T) + self.Q

        H_position = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        H_erp = np.array([[0, 0, 0, 0], [0, 0, 0, 0],
                         [0, 0, 1, 0], [0, 0, 0, 0]])
        H_imu = np.array([[0, 0, 0, 0], [0, 0, 0, 0],
                         [0, 0, 0, 0], [0, 0, 0, 1]])

        K_position = np.dot(np.dot(P_k, H_position.T),
                            self.inv(np.dot(np.dot(H_position, P_k), H_position.T) + R))
        K_erp = np.dot(np.dot(P_k, H_erp.T),
                       self.inv(np.dot(np.dot(H_erp, P_k), H_erp.T) + R))
        K_imu = np.dot(np.dot(P_k, H_imu.T),
                       self.inv(np.dot(np.dot(H_imu, P_k), H_imu.T) + R))

        X = x_k + np.dot(K_position, (z_position - np.dot(H_position, x_k))) + \
            np.dot(K_erp, (z_erp - np.dot(H_erp, x_k))) + \
            np.dot(K_imu, (z_imu - np.dot(H_imu, x_k)))

        self.x = X
        self.P = P_k - \
            np.dot(np.dot((K_position + K_erp + K_imu), np.identity(n=4)), P_k)

        self.dt = dt

        return self.x


class Sensor(object):
    def __init__(self, topic, msg_type):
        super(Sensor, self).__init__()
        self.data = np.array([0., 0., 0., 0.], dtype=np.float64)
        self.cov = np.array([
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ]
        ])
        self.sub = rospy.Subscriber(
            topic, msg_type, callback=self.sensorCallback)

    def sensorCallback(self, msg):
        self.data, self.cov = self.handleData(msg)

    def handleData(self, msg):
        return np.array([0., 0., 0., 0.], dtype=np.float64), self.cov


class ERP42(Sensor):
    def __init__(self, topic, msg_type):
        super(ERP42, self).__init__(topic, msg_type)
        self.cov = np.array([
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 99, 0., ],
            [0., 0., 0., 0., ]
        ])

    def handleData(self, msg):
        cov = getEmpty((4, 4))
        cov[2][2] = 0.5

        return np.array([0., 0., msg.speed, 0.], dtype=np.float64), cov


class Xsens(Sensor):
    def __init__(self, topic, msg_type):
        super(Xsens, self).__init__(topic, msg_type)
        self.cov = np.array([
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 99, ]
        ])

        self.initial_yaw = None
        self.yaw = 0.

    def handleData(self, msg):
        quat = msg.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        if self.initial_yaw is None:
            self.initial_yaw = yaw

        self.yaw = yaw - self.initial_yaw

        cov = getEmpty((4, 4))
        cov[-1][-1] = msg.orientation_covariance[-1]

        """
            [1.21847072356e-05, 0.0, 0.0, 
            0.0, 7.615442022250002e-05, 0.0, 
            0.0, 0.0, 0.00030461768089000006]
        """

        return np.array([0, 0, 0, self.yaw]), cov


class Odom(Sensor):
    def __init__(self, topic, msg_type):
        super(Odom, self).__init__(topic, msg_type)
        self.cov = np.array([
            [99., 0., 0., 0., ],
            [0., 99., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ]
        ])

    def handleData(self, msg):
        pose = msg.pose.pose.position

        odom_cov = np.reshape(
            np.array(msg.pose.covariance, dtype=np.float64), (6, 6))
        cov = odom_cov[:4, :4]

        return np.array([pose.x, pose.y, 0., 0.], dtype=np.float64), cov


if __name__ == "__main__":
    rospy.init_node("kalman")

    tf_listener = tf.TransformListener()
    pub = rospy.Publisher("test", PoseStamped, queue_size=1)

    gps = Odom("/odometry/gps", Odometry)
    hdl = Odom("/odom", Odometry)
    erp = ERP42("/erp42_feedback", SerialFeedBack)
    imu = Xsens("/imu/data", Imu)

    kf = Kalman()

    hz = 10
    dt = 1. / hz

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        res = kf.filter(gps, hdl, erp, imu,
                        dt=(current_time - last_time).to_sec())

        print(res)

        # msg = PoseStamped()

        # msg.header.frame_id = "odom"
        # msg.header.stamp = rospy.Time.now()

        # msg.pose.position.x = res[0]
        # msg.pose.position.y = res[1]

        # quat = quaternion_from_euler(0, 0, res[3])

        # msg.pose.orientation.x = quat[0]
        # msg.pose.orientation.y = quat[1]
        # msg.pose.orientation.z = quat[2]
        # msg.pose.orientation.w = quat[3]

        # pub.publish(msg)

        last_time = current_time

        r.sleep()
