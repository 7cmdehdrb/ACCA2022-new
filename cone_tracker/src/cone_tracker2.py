#!/usr/bin/env python


# Basic
import os
import sys
import rospy
import rospkg
import numpy as np
import math as m
from enum import Enum
from random import randint, random
import tf
from tf.transformations import quaternion_from_euler
from matplotlib import pyplot as plt
from random import randint
import threading
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
from grid_map import *


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


frame_id = "odom"


class Mapper(object):
    def __init__(self, xrange=[-10, 10], yrange=[-10, 10], size=0.25, test=False):
        self.frame_id = frame_id
        self.test = test

        self.xrange = xrange
        self.yrange = yrange
        self.size = size

        self.obstacles = []
        self.map = GridMap(self.xrange, self.yrange, self.size)

        # Subscribers
        rospy.Subscriber(
            "/adaptive_clustering/poses", PoseArray, self.obstacleCallback
        )
        self.tf_sub = tf.TransformListener()

    def obstacleCallback(self, msg):
        # Obstacle Callback. Automatically Calibrate and update self.obstacles
        self.current = rospy.Time.now()

        cones = []

        if self.test is True:
            for p in msg.poses:
                cone = [p.position.x, p.position.y]
                cones.append(cone)

        else:
            if self.tf_sub.canTransform(self.frame_id, "velodyne", rospy.Time(0)):
                for p in msg.poses:
                    dist = np.hypot(p.position.x, p.position.y)

                    if dist < 0.5 or dist > 10.0 or p.position.x < 0.0:
                        continue

                    pose = PoseStamped(
                        Header(None, rospy.Time(0), "velodyne"), p)
                    transformed_pose = self.tf_sub.transformPose(
                        self.frame_id, pose)

                    cone = [transformed_pose.pose.position.x,
                            transformed_pose.pose.position.y]
                    cones.append(cone)

            else:
                rospy.logwarn(
                    "Cannot lookup transform")

        self.mapping(cones)

    def mapping(self, cones):
        # Automatically Called.
        for new_cone in cones:
            flag = False
            idx = -1
            dist = -1

            for i, cone in enumerate(self.obstacles):
                dist = np.hypot(cone[0] - new_cone[0], cone[1] - new_cone[1])

                if dist < 0.5:
                    flag = True
                    calibrated_cone = [
                        (cone[0] + new_cone[0]) / 2.0, (cone[1] + new_cone[1]) / 2.0]
                    idx = i
                    break

            if flag is True:
                # new_cone is already existed.
                self.obstacles.pop(idx)
                self.obstacles += [calibrated_cone]

            elif (flag is False and dist > 0.) or len(self.obstacles) == 0:
                # find new cone
                self.obstacles += [new_cone]

        return self.obstacles


class PathPlanner(object):
    def __init__(self, state):
        self.state = state

        self.d_gain = 3.0
        self.c_gain = 15.0
        self.threshold = 0

        self.stack = 0

        self.test_pub = rospy.Publisher(
            "/cone_tracker/node", PointStamped, queue_size=1)
        self.node = Node(data=[self.state.x, self.state.y], parent=None)

        xrange = [-5, 50]
        yrange = [-30, 30]
        size = 0.25

        self.mapper = Mapper(xrange, yrange, size, False)

    def planning(self):
        if len(self.mapper.obstacles) < 4:
            rospy.logwarn("No Obstacle Datas")
            return PathResponse()

        min_cost = float("inf")
        node_dist = 0.0
        best_point = None

        points = self.getMiddlePoints(obstacles=self.mapper.obstacles)

        # state_vec = np.array(
        #     [m.cos(self.state.yaw), m.sin(self.state.yaw)])

        if self.node.parent is None:
            state_vec = np.array(
                [m.cos(self.state.yaw), m.sin(self.state.yaw)])

        else:
            state_vec = np.array([
                self.node.data[0] - self.node.parent.data[0],
                self.node.data[1] - self.node.parent.data[1]
            ])

        for point in points:
            p_curve, c_cost = self._calculateCurveCost(state_vec, point)

            if p_curve:
                p_distance, d_cost = self._calculateDistanceCost(point)

                if p_distance:
                    i, j = self.mapper.map.getGridIdx(point)

                    if self.mapper.map.map[i][j].count < 7:
                        grid_cost = self._calculateGridCost(point) * 0.1

                        total_cost = c_cost + d_cost + grid_cost

                        if total_cost < min_cost:
                            print(c_cost, d_cost, grid_cost, node_dist)
                            min_cost = total_cost
                            best_point = point

        if best_point is None or min_cost == float("inf"):
            rospy.logwarn("Cannot find best node...")

            for i in range(2):
                self.node.rollback()

            return createPathFromNode(self.node)

        new_node = Node(best_point, self.node)
        self.node = new_node
        self.node.findBestNode(self.mapper.map)

        self.test_pub.publish(self.node.parsePoint())

        return createPathFromNode(self.node)

    def _calculateDistanceCost(self, point):
        distance = np.hypot(
            self.node.data[0] - point[0], self.node.data[1] - point[1])

        b = (distance > 0.1 and distance < 3.0)

        return b, distance * self.d_gain

    def _calculateCurveCost(self, state_vec, point):
        point_vec = np.array([
            point[0] - self.node.data[0], point[1] - self.node.data[1]
        ])

        try:
            dot = np.dot(state_vec, point_vec)
            theta = abs(m.acos(
                (dot) / (np.hypot(state_vec[0], state_vec[1]) * np.hypot(point_vec[0], point_vec[1]))))

        except Exception as ex:
            print(point_vec)
            print(state_vec)
            rospy.logwarn(ex)
            return False, 0

        cost = theta * self.c_gain

        if theta >= (m.pi / 2.0) * 0.45:
            cost = float("inf")

        return dot > 0, cost

    def _calculateGridCost(self, point):
        total_cost = 0

        si, sj = self.mapper.map.getGridIdx([self.state.x, self.state.y])
        gi, gj = self.mapper.map.getGridIdx(point)

        dist_x = gi - si
        dist_y = gj - sj

        positive_x = dist_x >= 0
        positive_y = dist_y >= 0

        d = abs(abs(dist_x) - abs(dist_y))

        # Range Setting
        i = 0

        # Login Setting
        if abs(dist_x) >= abs(dist_y):
            for i in range(abs(dist_y)):
                cost = self.mapper.map.map[si +
                                           (i * (1 if positive_x else -1))][sj + (i * (1 if positive_y else -1))].count
                if cost >= 10:
                    return float("inf")
                total_cost += cost

            for j in range(d):
                cost = self.mapper.map.map[si +
                                           (i * (1 if positive_x else -1)) + (j * (1 if positive_x else -1))][sj + (i * (1 if positive_y else -1))].count
                if cost >= 10:
                    return float("inf")
                total_cost += cost

        else:
            for i in range(abs(dist_x)):
                cost = self.mapper.map.map[si +
                                           (i * (1 if positive_x else -1))][sj + (i * (1 if positive_y else -1))].count
                if cost >= 10:
                    return float("inf")
                total_cost += cost

            for j in range(d):
                cost = self.mapper.map.map[si +
                                           (i * (1 if positive_x else -1))][sj + (i * (1 if positive_y else -1)) + (j * (1 if positive_y else -1))].count
                if cost >= 10:
                    return float("inf")
                total_cost += cost

        return total_cost

    # def _createPath(self, goal):
    #     cx, cy, cyaw, _, _ = calc_spline_course(
    #         [self.state.x, goal[0]], [self.state.y, goal[1]], ds=0.1)
    #     return PathResponse(None, None, None, cx, cy, cyaw)

    def getMiddlePoints(self, obstacles):
        centers = []
        tri = Delaunay(obstacles)

        for ids in tri.simplices:
            center = self._getDelaunayCenters(obstacles, ids)
            centers += center

        centers = self._get_ordered_list(centers)

        return centers

    def _getDelaunayCenters(self, cones, ids):
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

    def _get_ordered_list(self, centers):
        centers = self._removeDuplicate(centers)
        sorted_point = sorted(
            centers, key=lambda e: self._getDistance(e, np.array([self.state.x, self.state.y])))

        return sorted_point

    def _getDistance(self, p1, p2):
        dist = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
        return dist

    def _removeDuplicate(self, centers):
        n = np.unique(centers, axis=0)
        return n

    def parsePath(self, path_response):
        path = Path()
        header = Header(None, rospy.Time.now(), frame_id)

        path.header = header

        for i in range(len(path_response.cx)):
            quat = quaternion_from_euler(0., 0., path_response.cyaw[i])

            p = PoseStamped()
            p.header = header
            p.pose.position = Point(
                path_response.cx[i], path_response.cy[i], 0.)
            p.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

            path.poses.append(p)

        return path


class Node(object):
    def __init__(self, data, parent=None):
        self.data = data
        self.parent = parent

    def findBestNode(self, map):
        # map = GridMap()
        i, j = map.getGridIdx(self.data)

        best_grid = map.map[i][j]
        best_cost = best_grid.count

        for a in range(-1, 2):
            for b in range(-1, 2):
                grid = map.map[i + a][j + b]
                new_cost = grid.count
                if new_cost < best_cost:
                    best_grid = grid
                    best_cost = new_cost

        if self.parent.data[0] == best_grid.x and self.parent.data[1] == best_grid.y:
            return False

        self.data = [best_grid.x, best_grid.y]
        print(self.data)
        return True

    def rollback(self):
        if self.parent is not None:
            self.data = self.parent.data
            self.parent = self.parent.parent

    def parsePoint(self):
        p = PointStamped(
            Header(None, rospy.Time.now(), frame_id),
            Point(self.data[0], self.data[1], 0.0)
        )
        return p


def createPathFromNode(node):
    xs = []
    ys = []

    while True:
        if node is None:
            break

        xs.append(node.data[0])
        ys.append(node.data[1])
        node = node.parent

    try:
        cx, cy, cyaw, _, _ = calc_spline_course(
            list(reversed(xs)), list(reversed(ys)), 0.1)

        return PathResponse(None, None, None, cx, cy, cyaw)

    except Exception as ex:
        rospy.logwarn("Cannot create path from nodes")
        rospy.logwarn(ex)

    return PathResponse()


def threadMapping():
    global planner, grid_pub

    while not rospy.is_shutdown():
        layer = GridMap(planner.mapper.xrange, planner.mapper.yrange,
                        planner.mapper.size, obstacles=planner.mapper.obstacles)
        planner.mapper.map = planner.mapper.map + layer
        grid_pub.publish(planner.mapper.map.parseOccupiedGrid())
        sleep(0.5)


if __name__ == "__main__":
    rospy.init_node("cone_tracker")

    state = State(odometry_topic="/odometry/kalman", hz=30, test=True)
    stanley = Stanley()

    sleep(3.0)

    planner = PathPlanner(state=state)

    grid_pub = rospy.Publisher(
        "/cone_tracker/gridmap", OccupancyGrid, queue_size=1)
    path_pub = rospy.Publisher("/cone_path", Path, queue_size=10)
    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)

    target_idx = 0

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = ControlMessage()
        msg.Gear = int(2)

        new_layer = GridMap(planner.mapper.xrange, planner.mapper.yrange,
                            planner.mapper.size, obstacles=planner.mapper.obstacles)
        planner.mapper.map += new_layer

        if grid_pub.get_num_connections() > 0:
            grid_pub.publish(planner.mapper.map.parseOccupiedGrid())

        path = planner.planning()

        try:
            di, target_idx = stanley.stanley_control(
                state, path.cx, path.cy, path.cyaw, last_target_idx=target_idx)
            di = np.clip(di, -m.radians(30), m.radians(30))

            msg.Speed = int(3)
            msg.Steer = int(m.degrees(-di))

        except Exception as ex:
            msg.Speed = int(0)
            # rospy.logwarn(ex)

        path_pub.publish(planner.parsePath(path))
        cmd_pub.publish(msg)

        r.sleep()
