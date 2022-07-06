#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import math as m
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from state import State
from cubic_spline_planner import calc_spline_course
from geometry_msgs.msg import PoseArray, Pose
import matplotlib.pyplot as plt

# class odom_path():
#     def __init__(self):
#         self.odometry = Odometry()
#         self.position_x = []
#         self.position_y = []
#         self.Yaw = []

#     def odometryCallBack(self, msg):
#         self.odometry = msg

#     def handledata(self):
#         pose_x = self.odometry.pose.pose.position.x
#         pose_y = self.odometry.pose.pose.position.y

#         x = self.odometry.pose.pose.orientation.x#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class State(object):
    def __init__(self, odometry_topic="/odometry/global"):

        # Subscriber
        self.odom_sub = rospy.Subscriber(
            odometry_topic, Odometry, callback=self.odomCallback)

        self.tf_sub = tf.TransformListener()

        # Custum Field
        self.data = Odometry()

        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s
        self.omega = 0.  # rad/s

        # Calculation
        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()

    def odomCallback(self, msg):
        self.currentTime = rospy.Time.now()

        msg = self.transformFrame(data=msg, target_frame="map")

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        dx = msg.pose.pose.position.x - self.data.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.data.pose.pose.position.y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        dt = (self.currentTime - self.lastTime).to_sec()

        self.v = distance / dt

        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quat)

        self.omega = (yaw - self.yaw) / dt
        self.yaw = yaw

        self.lastTime = self.currentTime

        self.data = msg

    def transformOdometryToPoseStamped(self, odom):
        pose = PoseStamped()

        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time(0)

        pose.pose = odom.pose.pose

        # print(odom)

        return pose

    def transformPoseStampedToOdometry(self, pose):
        odom = Odometry()

        odom.header.frame_id = pose.header.frame_id
        odom.header.stamp = rospy.Time(0)

        odom.pose.pose = pose.pose

        return odom

    def transformFrame(self, data, target_frame="map"):
        source_frame = data.header.frame_id

        try:
            if self.tf_sub.canTransform(target_frame=target_frame, source_frame=source_frame, time=rospy.Time(0)):
                pose = self.tf_sub.transformPose(
                    ps=self.transformOdometryToPoseStamped(data), target_frame=target_frame)
                odom = self.transformPoseStampedToOdometry(pose)
                return odom

            else:
                raise Exception()

        except Exception as ex:
            rospy.logwarn("Cannot Lookup Transform Between " +
                          target_frame + " and " + source_frame)
            rospy.logwarn(ex)
            return Odometry()

    def getArray(self):
        # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        return np.array([self.x, self.y, self.yaw, self.v, self.omega])

#         y = self.odometry.pose.pose.orientation.y
#         z = self.odometry.pose.pose.orientation.z
#         w = self.odometry.pose.pose.orientation.w

#         _, _, yaw = euler_from_quaternion([x, y, z, w])

#         self.position_x.append(pose_x)
#         self.position_y.append(pose_y)
#         self.Yaw.append(yaw)


def posePublish(cx, cy, cyaw):
    path = PoseArray()

    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "map"

    for i in range(0, len(cx)):
        pose = Pose()

        quat = quaternion_from_euler(0, 0, cyaw[i])

        pose.position.x = cx[i]
        pose.position.y = cy[i]
        pose.position.z = 0.0

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        path.poses.append(pose)

    Pub.publish(path)


if __name__ == "__main__":
    rospy.init_node("odom_path")

    Pub = rospy.Publisher("/create_global_path", PoseArray, queue_size=1)

    st = State()

    xs = []
    ys = []
    yaws = []

    # plt.show()
    # plt.savefig('drawing_path.png')

    r = rospy.Rate(1)
    while not rospy.is_shutdown():

        xs.append(st.x)
        ys.append(st.y)
        # yaws.append(state.yaw)

        new_xs = []
        new_ys = []
        # new_yaws = []

        for v in xs:
            if v not in new_xs:
                new_xs.append(v)

        for i in ys:
            if i not in new_ys:
                new_ys.append(i)

        # print(new_xs, new_ys)

        if len(new_xs) < 5:
            continue
        cx, cy, cyaw, _, _ = calc_spline_course(new_xs, new_ys)

        posePublish(cx, cy, cyaw)

        plt.plot(new_xs, new_ys)
        plt.pause(0.01)
        plt.savefig('drawing_path.png')

        r.sleep()
