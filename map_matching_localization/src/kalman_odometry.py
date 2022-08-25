#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, QuaternionStamped, Quaternion
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, QuaternionStamped, Quaternion, Point
from erp42_msgs.msg import SerialFeedBack
from erp42_control.msg import ControlMessage
from gaussian import *
from pyproj import *


is_publish_tf = rospy.get_param("/kalman/is_publish_tf", False)


def kph2mps(value):
    return value * 0.277778


def mps2kph(value):
    return value * 3.6


def getEmpty(shape):
    return np.array([[0. for i in range(shape[0])] for j in range(shape[1])])


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


class Kalman(object):
    def __init__(self, *args, **kwargs):
        self.odom_tf = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher(
            "/odometry/kalman", Odometry, queue_size=5)

        # initial value
        self.x = np.array([
            0, 0, 0, 0
        ])

        self.P = np.array([
            [1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]
        ])

        # noise
        self.Q = np.array([
            [0.5, 0., 0., 0.],
            [0., 0.5, 0., 0.],
            [0., 0., 0.01, 0.],
            [0., 0., 0., 0.1]
        ])

        self.dt = 0.

    def inv(self, matrix):
        return np.linalg.inv(matrix)

    def filter(self, gps, erp, imu, dt):
        A = np.array([
            [1, 0, m.cos(self.x[3]) * dt, 0],
            [0, 1, m.sin(self.x[3]) * dt, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        z_position = gps.data
        z_erp = erp.data
        z_imu = imu.data

        cov_position = gps.cov
        cov_erp = erp.cov
        cov_imu = imu.cov

        R = cov_position + cov_erp + cov_imu

        u_k = np.array(
            [0., 0., 0., (kph2mps(cmd.data[0]) / 1.040) * m.tan(-m.radians(cmd.data[1])) * dt])
        x_k = np.dot(A, self.x) + u_k
        P_k = np.abs(np.dot(np.dot(A, self.P), A.T)) + self.Q

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

        return self.x, self.P

    def publishOdom(self, x, P):
        msg = Odometry()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = x[0]
        msg.pose.pose.position.y = x[1]
        msg.pose.pose.position.z = 0

        quat = quaternion_from_euler(0, 0, x[3])

        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        msg.pose.covariance = [P[0][0], 0., 0., 0., 0., 0.,
                               0., P[1][1], 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., P[3][3]]

        self.odom_tf.sendTransform(
            translation=(x[0], x[1], 0.),
            rotation=quat,
            time=rospy.Time.now(),
            child="base_link",
            parent="odom"
        )

        self.odom_pub.publish(msg)


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
        self.once = False

        self.sub = rospy.Subscriber(
            topic, msg_type, callback=self.sensorCallback)

    def sensorCallback(self, msg):
        self.data, self.cov = self.handleData(msg)
        self.once = True

    def handleData(self, msg):
        return np.array([0., 0., 0., 0.], dtype=np.float64), self.cov


class ERP42(Sensor):
    def __init__(self, topic, msg_type):
        super(ERP42, self).__init__(topic, msg_type)
        self.cov = np.array([
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 999, 0., ],
            [0., 0., 0., 0., ]
        ])

        self.gear = 2

    def handleData(self, msg):
        self.gear = msg.Gear
        speed = msg.speed
        cov = getEmpty((4, 4))
        cov[2][2] = 0.01

        self.cov = cov

        return np.array([0., 0., speed, 0.], dtype=np.float64), self.cov


class Xsens(Sensor):
    def __init__(self, topic, msg_type):
        super(Xsens, self).__init__(topic, msg_type)

        self.cov = np.array([
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 99, ]
        ])

        self.yaw = 0.
        self.init_yaw = None

    def handleData(self, msg):
        quat = msg.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        if self.init_yaw is None:
            self.init_yaw = yaw

        self.yaw = normalize_angle(
            yaw - self.init_yaw + (np.pi if erp.gear == 2 else 0.0))

        # print("%.4f\t%.4f\t%.4f" % (yaw, self.init_yaw, self.yaw))

        cov = getEmpty((4, 4))
        cov[-1][-1] = msg.orientation_covariance[-1]

        self.cov = cov

        return np.array([0, 0, 0, self.yaw]), self.cov


class GPS(Sensor):
    def __init__(self, topic, msg_type):
        super(GPS, self).__init__(topic, msg_type)
        self.cov = np.array([
            [999., 0., 0., 0., ],
            [0., 999., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ]
        ])

        self.x = 0.
        self.y = 0.

        self.gps = Proj(init="epsg:4326")   # lat, log
        self.tm = Proj(init="epsg:2097")    # m

        self.last_position = None

    def calculateDistance(self, p1, p2):
        return m.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def calculateDistanceFromGPS(self, log, lat):
        x, y = transform(p1=self.gps, p2=self.tm,
                         x=log, y=lat)

        current_point = Point(x, y, 0.)

        if self.last_position is None:
            self.last_position = current_point

        distance = self.calculateDistance(self.last_position, current_point)

        self.last_position = current_point

        if distance < 0.012:
            distance = 0

        return distance

    def handleData(self, msg):
        distance = self.calculateDistanceFromGPS(
            log=msg.longitude, lat=msg.latitude)

        if imu.yaw is not None:
            self.x += distance * m.cos(imu.yaw)
            self.y += distance * m.sin(imu.yaw)

        # 37.5

        self.cov = np.array([
            [msg.position_covariance[0] *
                m.sqrt(111319.490793) + 37.5, 0., 0., 0., ],
            [0., msg.position_covariance[0] *
                m.sqrt(111319.490793) + 37.5, 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ]
        ])

        return np.array([self.x, self.y, 0., 0.], dtype=np.float64), self.cov


class Control(Sensor):
    def __init__(self, topic, msg_type):
        super(Control, self).__init__(topic, msg_type)

        self.data = np.array([0., 0., 0.])  # sp, st, br
        self.cov = None

    def handleData(self, msg):
        return np.array([msg.Speed, msg.Steer, msg.brake]), None


if __name__ == "__main__":
    rospy.init_node("kalman")

    gps = GPS("/ublox_gps/fix", NavSatFix)
    erp = ERP42("/erp42_feedback", SerialFeedBack)
    imu = Xsens("/imu/data", Imu)
    cmd = Control("/cmd_msg", ControlMessage)

    sensors = [
        gps, erp, imu
    ]

    kf = Kalman()

    hz = 30
    dt = 1. / hz

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(hz)

    # while not rospy.is_shutdown():
    #     is_all_available = True
    #     for s in sensors:
    #         if s.once is False:
    #             is_all_available = False
    #             break

    #     if is_all_available is True:
    #         break

    #     rospy.logwarn("Wait for Sensors...")

    #     r.sleep()

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        try:

            x, P = kf.filter(gps, erp, imu,
                             dt=(current_time - last_time).to_sec())

            kf.publishOdom(x, P)

        except Exception as ex:
            rospy.logwarn(ex)

        last_time = current_time

        r.sleep()
