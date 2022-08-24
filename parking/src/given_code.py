#!/usr/bin/env python

import sys
import rospy
import rospkg
import csv
import tf
import math as m
import numpy as np
import time as t
import threading
from std_msgs.msg import Int32MultiArray, Empty
from geometry_msgs.msg import PoseArray, Pose, Polygon, Point32, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/acca/catkin_ws/src")
PARKING_AREA_FILE = rospy.get_param("/parking_area", "parking.csv")

try:
    sys.path.insert(0, str(ACCA_FOLDER) + "/utils/")
    from loadPose import LoadPose
except ImportError as ie:
    print("UTIL IMPORT ERROR")
    print(ie)
    sys.exit()

"""

Subscribe 'adaptive_clustering/poses' topic and
Publish 'parking' topic to determine path

"""


L = 2.02
W = 1.12
RATIO = 1.7

mode = rospy.get_param("/save_parking", False)


def checkIsInParking(obstacle, box):
    dist = np.hypot(box.x - obstacle.x, box.y - obstacle.y)

    if dist == 0.0:
        return True

    area_VEC = np.array([
        m.cos(box.yaw - m.radians(90.0)), m.sin(box.yaw - m.radians(90.0))
    ])

    ob_VEC = np.array([
        obstacle.x - box.x, obstacle.y - box.y
    ])

    theta = m.acos(np.dot(area_VEC, ob_VEC) / dist)

    x_dist = abs(dist * m.sin(theta))
    y_dist = abs(dist * m.cos(theta))

    if x_dist <= box.x_len / 2.0 and y_dist <= box.y_len / 2.0:
        print(x_dist, y_dist)
        return True

    return False


class Obstacle(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Box(object):
    def __init__(self, x, y, yaw, x_len, y_len):
        super(Box, self).__init__()

        self.x = x
        self.y = y
        self.yaw = yaw
        self.x_len = x_len
        self.y_len = y_len

    def convertToPosition(self):

        point1 = (self.x + m.cos(self.yaw), self.y + m.sin(self.yaw))
        point2 = (self.x - m.cos(self.yaw), self.y + m.sin(self.yaw))
        point3 = (self.x - m.cos(self.yaw), self.y - m.sin(self.yaw))
        point4 = (self.x + m.cos(self.yaw), self.y - m.sin(self.yaw))

        return [point1, point2, point3, point4]


class CarYN(object):
    def __init__(self):
        super(CarYN, self).__init__()
        self.boxes = []
        self.obstacles = []
        self.parkspace = []

        self.deleteFlag = True

        self.tf_node = tf.TransformListener()

    def CarCallback(self, msg):
        # centpo = msg.poses
        temp_array = []

        centpo = msg.poses

        for p in centpo:
            pose = PoseStamped()

            pose.header.frame_id = "velodyne"
            pose.header.stamp = rospy.Time(0)

            pose.pose.position.x = p.position.x
            pose.pose.position.y = p.position.y

            pose.pose.orientation.w = 1.0

            map_pose = self.tf_node.transformPose("map", pose)

            new_ob = Obstacle(x=map_pose.pose.position.x,
                              y=map_pose.pose.position.y)

            temp_array.append(new_ob)

        self.obstacles = temp_array

    def parkingPositionCallback(self, msg):
        # msg = PoseStamped()

        x = msg.pose.position.x
        y = msg.pose.position.y

        ori_x = msg.pose.orientation.x
        ori_y = msg.pose.orientation.y
        ori_z = msg.pose.orientation.z
        ori_w = msg.pose.orientation.w

        _, _, YAW = tf.transformations.euler_from_quaternion(
            [ori_x, ori_y, ori_z, ori_w])

        x_len = W * RATIO
        y_len = L * RATIO

        box = Box(x=x, y=y, yaw=YAW + m.radians(90.0),
                  x_len=x_len, y_len=y_len)

        print("ADD NEW BOX")

        self.boxes.append(box)

    def obstacleTestCallback(self, msg):

        x = msg.pose.position.x
        y = msg.pose.position.y

        new_obstacle = Obstacle(x=x, y=y)

        print("ADD NEW OBSTACLE")

        self.obstacles.append(new_obstacle)

    def checkingParking(self):
        result = []

        for box in self.boxes:
            # box = Box()

            flag = 0
            points = box.convertToPosition()

            for obstacle in self.obstacles:
                # obstacle = Obstacle()
                P = [obstacle.x, obstacle.y]

                # res = isPointInPath(P=P, poly=points)
                res = checkIsInParking(obstacle=obstacle, box=box)

                if res is True:
                    flag = 1

            result.append(flag)

        return result

    def publishMarker(self, publisher):
        msg = MarkerArray()

        idx = 0

        # print(len(self.boxes))

        for box in self.boxes:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.lifetime = rospy.Duration(0.2)
            marker.ns = str(idx)

            marker.type = marker.CUBE
            marker.action = marker.ADD

            # box = Box()

            marker.scale.x = box.x_len
            marker.scale.y = box.y_len
            marker.scale.z = 0.1

            marker.color.a = 0.5
            marker.color.r = 1.0 if idx == 0 or idx == 3 else 0.0
            marker.color.g = 1.0 if idx == 1 or idx == 3 else 0.0
            marker.color.b = 1.0 if idx == 2 or idx == 3 else 0.0

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, box.yaw)

            marker.pose.position.x = box.x
            marker.pose.position.y = box.y
            marker.pose.position.z = 0.0

            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

            msg.markers.append(marker)

            idx += 1

        publisher.publish(msg)

    def publishObstacle(self, publisher):
        msg = PoseArray()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.poses = []

        for obstacle in self.obstacles:
            pose = Pose()

            pose.position.x = obstacle.x
            pose.position.y = obstacle.y
            pose.position.z = 0.0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            msg.poses.append(pose)

        publisher.publish(msg)

    def saveParking(self):
        rospy.wait_for_message("/save_parking", Empty)
        rospy.loginfo("TRYING TO SAVE PARKING...")

        output_file_path = rospkg.RosPack().get_path(
            'parking')+"/saved_data/" + PARKING_AREA_FILE

        with open(output_file_path, 'w') as csvfile:
            for box in self.boxes:
                # box = Box()

                text = ""

                text += str(box.x)
                text += ","
                text += str(box.y)
                text += ","
                text += str(box.yaw)
                text += ","
                text += str(box.x_len)
                text += ","
                text += str(box.y_len)
                text += "\n"

                csvfile.write(text)

        rospy.loginfo("SAVING FINISHED")

    def deleteOne(self):
        while not rospy.is_shutdown():
            if self.deleteFlag is True:
                rospy.wait_for_message("/delete_area", Empty)
                rospy.loginfo("DELETE LATEST POINT... PLZ WAIT")

                self.boxes.pop()

                self.deleteFlag = False

                t.sleep(5.0)

                self.deleteFlag = True

                rospy.loginfo("SUCCESS")

    def loadParking(self):
        output_file_path = rospkg.RosPack().get_path(
            'parking')+"/saved_data/" + PARKING_AREA_FILE

        rospy.loginfo("LOADING CSV FILE...")
        with open(output_file_path, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")
            for row in reader:

                box = Box(
                    x=float(row[0]),
                    y=float(row[1]),
                    yaw=float(row[2]),
                    x_len=float(row[3]),
                    y_len=float(row[4]),
                )

                self.boxes.append(box)

        rospy.loginfo("LOADING FINISHED")


if __name__ == "__main__":
    rospy.init_node("parking_selection")

    caryn = CarYN()

    park_pub = rospy.Publisher("parking", Int32MultiArray, queue_size=1)
    marker_pub = rospy.Publisher("parking_area", MarkerArray, queue_size=1)

    rospy.Subscriber("adaptive_clustering/poses", PoseArray, caryn.CarCallback)

    """ TEST """

    rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                     callback=caryn.obstacleTestCallback)
    # obstacle_pub = rospy.Publisher("/obstacle_test", PoseArray, queue_size=1)

    """ TEST END """

    if mode is True:
        rospy.loginfo("RUNNING SAVE MODE")
        rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                         callback=caryn.parkingPositionCallback)

        th1 = threading.Thread(target=caryn.saveParking)
        th2 = threading.Thread(target=caryn.deleteOne)
        th1.start()
        th2.start()

        lp = LoadPose(file_name="kcity_parking2.csv")
        tpub = rospy.Publisher("test_parking", Path, queue_size=10)

    else:
        rospy.loginfo("RUNNING LOAD MODE")
        caryn.loadParking()

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():

        data = Int32MultiArray()
        data.data = caryn.checkingParking()

        caryn.publishMarker(publisher=marker_pub)
        park_pub.publish(data)

        # caryn.publishObstacle(publisher=obstacle_pub) # TEST

        # print(data.data)
        # lp.pathPublish(pub=tpub)

        r.sleep()
