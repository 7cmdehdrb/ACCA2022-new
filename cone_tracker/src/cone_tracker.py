#!/usr/bin/env python


# Basic
import os
import sys
import rospy
import rospkg
import numpy as np
import math as m
import genpy
from enum import Enum
from random import randint, random
import tf
from tf.transformations import quaternion_from_euler
from time import sleep

# msg
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from erp42_control.msg import *
from path_plan.msg import PathResponse

# Custom
from scipy.spatial import Delaunay

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State, OdomState
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)


try:
    path_plan_pkg_path = rospkg.RosPack().get_path("path_plan") + "/src"
    sys.path.append(path_plan_pkg_path)
    from cubic_spline_planner import calc_spline_course
except Exception as ex:
    rospy.logfatal(ex)


frame_id = "map"


class Cone(object):
    def __init__(self, x, y, r=0.2):
        self.x = x
        self.y = y
        self.r = r

    def __add__(self, other):
        x = (self.x + other.x) / 2.
        y = (self.y + other.y) / 2.
        r = (self.r + other.r) / 2.
        return Cone(x, y, r)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and other.r and other.r


class Mapper(object):
    def __init__(self):
        self.cones = []

        self.cone_pub = rospy.Publisher(
            "/cone_tracker/cones", PoseArray, queue_size=1)

    def publishCones(self):
        msg = PoseArray()
        header = Header(None, rospy.Time.now(), frame_id)

        msg.header = header
        for cone in self.cones:
            p = Pose()

            p.position.x = cone.x
            p.position.y = cone.y

            msg.poses.append(p)

        self.cone_pub.publish(msg)

    def mapping(self, cones):
        for new_cone in cones:
            flag = False
            idx = -1
            dist = -1
            for i, cone in enumerate(self.cones):
                dist = np.hypot(cone.x - new_cone.x, cone.y - new_cone.y)
                if dist < cone.r or dist < new_cone.r:
                    flag = True
                    calibrated_cone = new_cone + cone
                    idx = i
                    break

            if flag is True:
                # new_cone is already existed.
                self.cones.pop(idx)
                self.cones += [calibrated_cone]

            elif (flag is False and dist > 0.) or len(self.cones) == 0:
                # find new cone
                rospy.loginfo("New cone detected! : %.4f" % dist)
                self.cones += [new_cone]

        return self.cones

    def parseArray(self):
        res = []
        for cone in self.cones:
            res.append(np.array([cone.x, cone.y]))
        return np.array(res)

    def calculateMiddlePoints(self, state):
        centers = []
        cones = self.parseArray()
        tri = Delaunay(cones)

        for ids in tri.simplices:
            center = self.getDelaunayCenters(cones, ids)
            centers += center

        centers = self.get_ordered_list(centers, state)

        return centers

    def getDelaunayCenters(self, cones, ids):
        res = []

        p0 = cones[ids[0]]
        p1 = cones[ids[1]]
        p2 = cones[ids[2]]

        # 1
        x = (p0[0] + p1[0]) / 2.
        y = (p0[1] + p1[1]) / 2.

        res.append([x, y])

        # 2
        x = (p0[0] + p2[0]) / 2.
        y = (p0[1] + p2[1]) / 2.

        res.append([x, y])

        # 3
        x = (p1[0] + p2[0]) / 2.
        y = (p1[1] + p2[1]) / 2.

        res.append([x, y])

        return res

    def get_ordered_list(self, centers, state):
        centers = self.removeDuplicate(centers)
        sorted_point = sorted(
            centers, key=lambda e: self.getDistance(e, np.array([state.x, state.y])))

        return sorted_point

    def getDistance(self, p1, p2):
        dist = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
        return dist

    def removeDuplicate(self, centers):
        n = np.unique(centers, axis=0)
        return n


class PathPlanner(object):
    def __init__(self, state, d_gain, c_gain, threshold):
        self.state = state
        self.d_gain = d_gain
        self.c_gain = c_gain
        self.threshold = threshold

    def planning(self, points):
        state_vec = np.array([m.cos(self.state.yaw), m.sin(self.state.yaw)])

        min_cost = float("inf")
        best_path = PathResponse(None, None, None, [], [], [])

        for point in points:
            cost = self.calculateCost(state_vec, point)

            if cost < self.threshold:
                return self.createPath(goal=point)

            if cost < min_cost:
                min_cost = cost
                best_path = self.createPath(goal=point)

        return best_path

    def createPath(self, goal):
        cx, cy, cyaw, _, _ = calc_spline_course([self.state.x, goal[0]], [
            self.state.y, goal[1]], ds=0.1)
        return PathResponse(None, None, None, cx, cy, cyaw)

    def calculateDistanceCost(self, state_vec, point):
        distance = np.hypot(self.state.x - point[0], self.state.y - point[1])

        if distance < 3.0:
            return float("inf")

        return distance * self.d_gain

    def calculateCurveCost(self, state_vec, point):
        point_vec = np.array([
            point[0] - self.state.x, point[1] - self.state.y
        ])

        # 0.0 ~
        theta = abs(m.acos((np.dot(state_vec, point_vec)) /
                           (np.hypot(state_vec[0], state_vec[1]) * np.hypot(point_vec[0], point_vec[1]))))

        if theta >= (m.pi / 2.0) * 0.6:
            return float("inf")

        return theta * self.c_gain

    def calculateCost(self, state_vec, obstacle):
        distance_cost = self.calculateDistanceCost(state_vec, obstacle)
        curve_cost = self.calculateCurveCost(state_vec, obstacle)

        total = distance_cost + curve_cost

        if total < 9999.:
            print(distance_cost, curve_cost)

        return distance_cost + curve_cost


class ConeTracker(object):
    def __init__(self):
        # Subscriber
        self.cone_sub = rospy.Subscriber(
            "/cone_simulator/detected_cones", PoseArray, self.coneCallback)
        self.tf_sub = tf.TransformListener()

        # Map
        self.map = Mapper()

    def coneCallback(self, msg, test=True):
        cones = []

        if len(msg.poses) > 50:
            return 0

        if test is True:
            for p in msg.poses:
                cone = Cone(p.position.x, p.position.y, 0.5)
                cones.append(cone)

        else:
            if self.tf_sub.canTransform(frame_id, "velodyne", rospy.Time(0)):
                for p in msg.poses:
                    pose = PoseStamped(
                        Header(None, rospy.Time(0), "velodyne"), p)
                    transformed_pose = self.tf_sub.transformPose(
                        frame_id, pose)

                    cone = Cone(transformed_pose.pose.position.x,
                                transformed_pose.pose.position.y, 1.0)
                    cones.append(cone)

            else:
                rospy.logwarn(
                    "Cannot lookup transform")

        self.map.mapping(cones)


def parsePath(path_response=PathResponse()):
    path = Path()
    header = Header(None, rospy.Time.now(), frame_id)

    path.header = header

    for i in range(len(path_response.cx)):
        quat = quaternion_from_euler(0., 0., path_response.cyaw[i])

        p = PoseStamped()
        p.header = header
        p.pose.position = Point(path_response.cx[i], path_response.cy[i], 0.)
        p.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        path.poses.append(p)

    return path


if __name__ == "__main__":
    rospy.init_node("cone_tracker")

    d_gain = 1.0
    c_gain = 4.0
    threshold = 0.0

    state = State(odometry_topic="odometry/kalman", hz=30, test=True)
    stanley = Stanley()
    s_target_idx = 0

    path_pub = rospy.Publisher("/cone_path", Path, queue_size=1)
    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)
    msg = ControlMessage()

    cone_tracker = ConeTracker()
    path_planner = PathPlanner(
        state=state, d_gain=d_gain, c_gain=c_gain, threshold=threshold)

    # sleep(5)

    r = rospy.Rate(30)
    while not rospy.is_shutdown():

        if len(cone_tracker.map.cones) < 10:
            r.sleep()
            continue

        cone_tracker.map.publishCones()

        path = path_planner.planning(
            cone_tracker.map.calculateMiddlePoints(state))
        path_pub.publish(parsePath(path))

        try:
            di, _ = stanley.stanley_control(
                state, path.cx, path.cy, path.cyaw, last_target_idx=len(path.cx)-1)
            di = np.clip(di, -m.radians(30), m.radians(30))

            msg.Speed = int(3)
            msg.Steer = int(m.degrees(-di))
            msg.Gear = int(2)

            cmd_pub.publish(msg)

        except Exception as ex:
            rospy.logwarn(ex)

        r.sleep()
