#!/usr/bin/env python


from numpy import average
import rospy
import rospkg
import math as m
import numpy as np
import time as t
import threading
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from point_cloud import array_to_xyz_pointcloud2f
import pygicp


global_map_topic = rospy.get_param("global_map_topic", "cloud_pcd")
distance_threshold = rospy.get_param("map_matching_threshold", 25)


class PointCloud(object):
    def __init__(self, pcd_topic, oversample=False):
        self.topic = pcd_topic

        self.doOversample = oversample

        self.data = None
        self.sliced_data = None
        self.slicing_point = None  # PoseStamped

        self.pcd_subscriber = rospy.Subscriber(
            pcd_topic, PointCloud2, callback=self.pointCloudCallback)

    def pointCloudCallback(self, msg):
        temp = list(read_points(cloud=msg, field_names=(
            "x", "y", "z"), skip_nans=True))

        if self.topic == global_map_topic:
            if self.data is None:
                self.data = temp[::10]
                # self.data = temp
                rospy.loginfo(
                    "GlobalMap is received! %d Point clouds" % len(self.data))
            else:
                rospy.loginfo_once(
                    "GlobalMap is received but ignored because data is already taken")
                return

        else:
            if self.doOversample is True:
                # self.data = self.overSampling([i for i in temp if i[2] > -0.7])
                self.data = self.overSampling(temp)

                del temp
            else:
                # self.data = temp
                self.data = [i for i in temp if i[2] > -0.7]

                del temp

    def overSampling(self, data):
        if data is None:
            rospy.logwarn("Oversample Error : No Data")
            return None

        res = []

        for i in range(int(len(data) / 2 - 1)):
            first = data[2*i]  # 0, 2, 4, 6...
            second = data[2*i+1]  # 1, 3, 5, 7...

            x = (first[0] + second[0]) / 2.0
            y = (first[1] + second[1]) / 2.0
            z = (first[2] + second[2]) / 2.0

            res.append(first)
            res.append([x, y, z])
            res.append(second)

        return res

    def slicePointCloud(self):
        if self.slicing_point is None:
            rospy.logwarn("NO SLICING POINT")
            return 0

        if self.data is None:
            rospy.logwarn("NO Point Clouds")
            return 0

        new_data = []

        ix = self.slicing_point.pose.position.x
        iy = self.slicing_point.pose.position.y
        iz = self.slicing_point.pose.position.z

        for p in self.data:
            x = p[0]
            y = p[1]
            z = p[2]

            distance = m.sqrt((ix - x) ** 2 + (iy - y) ** 2)

            if distance < distance_threshold:
                new_data.append(p)

        self.sliced_data = new_data

    def setSlicingPoint(self, point):
        if type(point) != type(PoseStamped()):
            rospy.logwarn("Slicing point type ERROR")
            return 0

        self.slicing_point = point


def doMapMatching(target, source):
    # Exception
    if (target) is None or (source) is None:
        rospy.logwarn("No Target or Source Matrix")
        return np.array(
            [[0, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 1]]
        )

    target = pygicp.downsample(target, 0.5)
    source = pygicp.downsample(source, 0.5)

    gicp = pygicp.FastGICP()
    gicp.set_input_target(target)
    gicp.set_input_source(source)

    gicp.set_num_threads(8)
    # gicp.set_max_correspondence_distance(100.0)

    gicp.align()

    tf_matrix = gicp.get_final_transformation()

    return tf_matrix


def doTransform(source, tf_matrix):
    if source is None or tf_matrix is None:
        rospy.logwarn("No Source or TF Matrix")
        return []

    res = []

    for i in source:
        before_tf = np.array([i[0], i[1], i[2], 1])
        after_tf = np.dot(tf_matrix, before_tf)
        res.append([after_tf[0], after_tf[1], after_tf[2]])

    return res


class MapOdomTF(object):
    def __init__(self, hz=100):
        self.init_pose_subscriber = rospy.Subscriber(
            "initialpose", PoseWithCovarianceStamped, callback=self.initPoseCallback)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.hz = hz

        self.init_pose = None
        self.trans_pose = None

        th = threading.Thread(target=self.run)
        th.start()

    def initPoseCallback(self, msg):
        frame_id = msg.header.frame_id

        # Exception
        if frame_id != "map":
            rospy.logerr("RVIZ fixed frame is not map")
            rospy.logerr("Make sure your fixed frame is map")
            return 0

        rospy.loginfo("Initial pose detected")

        self.trans_pose = None
        self.init_pose = msg

    def createNewTF(self, tf_matrix):
        # base_link > map
        zero = [0, 0, 0, 0]
        zero_t = doTransform([zero], tf_matrix)[0]

        zero_t_pose = PoseStamped()
        zero_t_pose.header.frame_id = "map"
        zero_t_pose.header.stamp = rospy.Time.now() - rospy.Duration(0.1)

        zero_t_pose.pose.position.x = zero_t[0]
        zero_t_pose.pose.position.y = zero_t[1]
        zero_t_pose.pose.position.z = zero_t[2]

        zero_t_pose.pose.orientation.w = 1.0

        # base_link > map 4x4
        zero = [0, 0, 0, 0]
        one = [1, 0, 0, 0]

        # map > base_link 4x4
        rev_tf_matrix = np.linalg.inv(tf_matrix)

        test = doTransform([[0, 0, 0, 0]], rev_tf_matrix)[0]
        print(test)

        return
        tf_zero = doTransform([zero], rev_tf_matrix)[0]
        tf_one = doTransform([one], rev_tf_matrix)[0]

        # map > base_link trans and rot matrix
        trans = tf_zero
        theta = np.dot(tf_one, tf_zero)
        rot = tf.transformations.quaternion_from_euler(0, 0, theta)

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = rospy.Time.now() - rospy.Duration(0.1)
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]

        if self.tf_listener.canTransform("base_link", "odom", rospy.Time.now() - rospy.Duration(0.1)):
            new_pose = self.tf_listener.transformPose("odom", pose)
            if self.checkValidation(self.init_pose, new_pose) is True:
                self.trans_pose = new_pose
            else:
                rospy.logwarn("New TF has problem...")

            return new_pose

        else:
            rospy.logwarn(
                "NO TF data from base_link to odom")
            return PoseStamped()

    def checkValidation(self, init_pose, trans_pose):

        if init_pose is None or trans_pose is None:
            return False

        position1 = init_pose.pose.pose.position
        position2 = trans_pose.pose.position

        dist = m.sqrt((position1.x - position2.x) ** 2 +
                      (position1.y - position2.y) ** 2)

        if dist < 8.0:
            return True

        return False

    def publishTF(self):

        if self.init_pose is None:
            rospy.logwarn(
                "NO initial pose for TF relation between map and odom")
            return 0

        if self.trans_pose is None:
            trans = (self.init_pose.pose.pose.position.x,
                     self.init_pose.pose.pose.position.y, self.init_pose.pose.pose.position.z)
            quat = [self.init_pose.pose.pose.orientation.x, self.init_pose.pose.pose.orientation.y,
                    self.init_pose.pose.pose.orientation.z, self.init_pose.pose.pose.orientation.w]

            self.tf_broadcaster.sendTransform(
                trans, quat, rospy.Time.now(), "odom", "map")

            return 0

        else:
            rospy.loginfo_once("START broadcasting tf with GICP")

            trans = (self.trans_pose.pose.position.x,
                     self.trans_pose.pose.position.y, self.trans_pose.pose.position.z)
            quat = [self.trans_pose.pose.orientation.x, self.trans_pose.pose.orientation.y,
                    self.trans_pose.pose.orientation.z, self.trans_pose.pose.orientation.w]

            self.tf_broadcaster.sendTransform(
                trans, quat, rospy.Time.now(), "odom", "map")

            return 0

    def run(self):
        r = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.publishTF()
            r.sleep()


class MapMatchingOdometry(object):
    def __init__(self, odom_topic):

        self.odom_subscriber = rospy.Subscriber(
            odom_topic, Odometry, self.odomCallback)
        self.odom_tf_subscriber = tf.TransformListener()

        self.data = None

    def odomCallback(self, msg):
        self.data = msg

    def transformOdometry2PoseStamped(self):
        if self.data is None:
            rospy.logwarn(
                "NO odometry data")
            return PoseStamped()

        transformed_data = PoseStamped()

        transformed_data.header = self.data.header
        transformed_data.pose.position = self.data.pose.pose.position
        transformed_data.pose.orientation = self.data.pose.pose.orientation

        return transformed_data

    def getOdomOnMap(self):
        if self.data is None:
            rospy.logwarn(
                "NO odometry data")
            return 0

        if not self.odom_tf_subscriber.canTransform("map", "odom", rospy.Time.now() - rospy.Duration(1)):
            rospy.logwarn(
                "NO TF data from map to odom")
            return 0

        transformed_data = PoseStamped()

        transformed_data.header.frame_id = self.data.header.frame_id
        transformed_data.header.stamp = rospy.Time.now() - rospy.Duration(0.1)

        transformed_data.pose.position = self.data.pose.pose.position
        transformed_data.pose.orientation = self.data.pose.pose.orientation

        transformed_pose = self.odom_tf_subscriber.transformPose(
            "map", transformed_data)

        return transformed_pose
