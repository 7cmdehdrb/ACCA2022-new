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
from hdl_localization.msg import ScanMatchingStatus
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


class Kalman(object):
    def __init__(self, *args, **kwargs):
        self.odom_pub = rospy.Publisher(
            "/odometry/kalman", Odometry, queue_size=1)

        self.tf_br = tf.TransformBroadcaster()

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
            [0., 0., 0., 0.01]
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

        z_gps = [
            Gaussian(None, gps.data[0], gps.cov[0][0]),
            Gaussian(None, gps.data[1], gps.cov[1][1])
        ]    # [x, y]

        z_hdl = [
            Gaussian(None, hdl.data[0], hdl.cov[0][0]),
            Gaussian(None, hdl.data[1], hdl.cov[1][1])
        ]    # [x, y]

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

        # u_k = np.array(
        #     [0., 0., 0., (kph2mps(cmd.data[0]) / 1.040) * m.tan(-m.degrees(cmd.data[1])) * dt])
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

        return self.x, self.P

    def publishOdom(self, x, P):
        msg = Odometry()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
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

        self.odom_pub.publish(msg)

        if is_publish_tf is False:
            self.tf_br.sendTransform(translation=(
                x[0], x[1], 0), rotation=quat, time=rospy.Time.now(), child='base_link', parent='map')


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

    def handleData(self, msg):
        cov = getEmpty((4, 4))
        cov[2][2] = 0.01

        self.cov = cov

        return np.array([0., 0., msg.speed, 0.], dtype=np.float64), self.cov


class Xsens(Sensor):
    def __init__(self, topic, msg_type):
        super(Xsens, self).__init__(topic, msg_type)

        self.flag_sub = rospy.Subscriber(
            "/set_local", Empty, callback=self.set_imu)
        self.flag_pub = rospy.Publisher(
            "/set_local", Empty, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            "/odom", Odometry, callback=self.odomCallback)
        self.status_sub = rospy.Subscriber(
            "/status", ScanMatchingStatus, self.statusCallback
        )

        self.cov = np.array([
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 99, ]
        ])

        self.imu_flag = False
        self.last_yaw = None
        self.yaw = None

        self.current = rospy.Time.now()
        self.last = rospy.Time.now()

    def odomCallback(self, msg):
        if self.imu_flag is True:
            quat = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.yaw = yaw
            self.imu_flag = False

    def statusCallback(self, msg):
        if msg.matching_error < 0.02:
            self.flag_pub.publish()
            self.status_sub.unregister()

    def set_imu(self, msg):
        self.imu_flag = True
        rospy.loginfo("SET INITIAL IMU!")

    def handleData(self, msg):

        self.current = rospy.Time.now()
        dt = (self.current - self.last).to_sec()
        hz = 1 / dt

        self.last = self.current

        if self.yaw is None:
            return np.array([0., 0., 0., 0.], dtype=np.float64), self.cov

        if hz > 450:
            return np.array([0, 0, 0, self.yaw]), self.cov

        quat = msg.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        if self.yaw is not None:

            if self.last_yaw is None:
                self.last_yaw = yaw

            dyaw = yaw - self.last_yaw
            self.yaw += dyaw

            cov = getEmpty((4, 4))
            cov[-1][-1] = msg.orientation_covariance[-1]

            self.cov = cov

            self.last_yaw = yaw

            return np.array([0, 0, 0, self.yaw]), self.cov


class HDL(Sensor):
    def __init__(self, topic, msg_type):
        super(HDL, self).__init__(topic, msg_type)
        self.cov = np.array([
            [999., 0., 0., 0., ],
            [0., 999., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ]
        ])

        self.x = 0.
        self.y = 0.
        self.quat = Quaternion(0., 0., 0., 1.)

    def handleData(self, msg):
        pose = msg.pose.pose.position

        self.x = pose.x
        self.y = pose.y
        self.quat = msg.pose.pose.orientation

        odom_cov = np.reshape(
            np.array(msg.pose.covariance, dtype=np.float64), (6, 6))
        cov = odom_cov[:4, :4]

        if cov[0][0] > 1.0:
            for i in range(2):
                cov[i][i] = 9999.

        self.cov = cov

        return np.array([pose.x, pose.y, 0., 0.], dtype=np.float64), self.cov


class GPS(Sensor):
    def __init__(self, topic, msg_type):
        super(GPS, self).__init__(topic, msg_type)
        self.cov = np.array([
            [999., 0., 0., 0., ],
            [0., 999., 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ]
        ])

        self.flag_sub = rospy.Subscriber(
            "/set_local", Empty, callback=self.set_gps)
        self.flag_pub = rospy.Publisher(
            "/set_local", Empty, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            "/odom", Odometry, callback=self.odomCallback)
        self.status_sub = rospy.Subscriber(
            "/status", ScanMatchingStatus, self.statusCallback
        )
        self.gps_odom_pub = rospy.Publisher(
            "odometry/gps", Odometry, queue_size=1)

        self.x = 0.
        self.y = 0.

        self.gps = Proj(init="epsg:4326")   # lat, log
        self.tm = Proj(init="epsg:2097")    # m
        # self.tm = Proj(init="epsg:5174")    # m

        self.gps_flag = False
        self.last_position = None

    def odomCallback(self, msg):
        if self.gps_flag is True:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y

            kf.x[0] = self.x
            kf.x[1] = self.y

            self.gps_flag = False

    def statusCallback(self, msg):
        if msg.matching_error < 0.02:
            self.flag_pub.publish()
            self.status_sub.unregister()

    def set_gps(self, msg):
        self.gps_flag = True
        rospy.loginfo("SET INITIAL GPS!")

    def publishOdometry(self, x, y):
        msg = Odometry()

        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        quat = quaternion_from_euler(0., 0., kf.x[3])

        msg.pose.pose.orientation = Quaternion(
            quat[0], quat[1], quat[2], quat[3])

        msg.pose.covariance = [0. for i in range(36)]
        msg.pose.covariance[0] = self.cov[0][0]
        msg.pose.covariance[7] = self.cov[1][1]

        self.gps_odom_pub.publish(msg)

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

        x = self.x
        y = self.y

        self.cov = np.array([
            [msg.position_covariance[0] *
                m.sqrt(111319.490793) + 37.5, 0., 0., 0., ],
            [0., msg.position_covariance[0] *
                m.sqrt(111319.490793) + 37.5, 0., 0., ],
            [0., 0., 0., 0., ],
            [0., 0., 0., 0., ]
        ])

        self.publishOdometry(x, y)

        return np.array([x, y, 0., 0.], dtype=np.float64), self.cov


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
    hdl = HDL("/odom", Odometry)
    erp = ERP42("/erp42_feedback", SerialFeedBack)
    imu = Xsens("/imu/data", Imu)
    cmd = Control("/cmd_msg", ControlMessage)

    kf = Kalman()

    hz = 10
    dt = 1. / hz

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        try:
            x, P = kf.filter(gps, hdl, erp, imu,
                             dt=(current_time - last_time).to_sec())

            kf.publishOdom(x, P)

        except Exception as ex:
            rospy.logwarn(ex)

        last_time = current_time

        r.sleep()
