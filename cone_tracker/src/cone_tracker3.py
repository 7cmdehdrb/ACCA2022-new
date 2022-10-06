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
    from pid_tuner import PID
except Exception as ex:
    rospy.logfatal(ex)


try:
    path_plan_pkg_path = rospkg.RosPack().get_path("path_plan") + "/src"
    sys.path.append(path_plan_pkg_path)
    from cubic_spline_planner import calc_spline_course
except Exception as ex:
    rospy.logfatal(ex)


frame_id = "odom"
print_tendency = False


class Mapper(object):
    def __init__(self, xrange=[-10, 10], yrange=[-10, 10], size=0.25, test=True):
        self.frame_id = frame_id
        self.test = test

        self.xrange = xrange
        self.yrange = yrange
        self.size = size

        self.obstacles = []
        self.map = GridMap(self.xrange, self.yrange, self.size)

        topic = "/cone_simulator/detected_cones"
        clustering_topic = "/adaptive_clustering/poses"

        self.tendency = 0.0

        # Subscribers
        rospy.Subscriber(
            topic, PoseArray, self.obstacleCallback
        )
        rospy.Subscriber(
            clustering_topic, PoseArray, self.clusteringCallback
        )
        
        self.tf_sub = tf.TransformListener()

    def clusteringCallback(self, msg):
        tendency = 0.0

        for p in msg.poses:
            if p.position.x < 0.0 or p.position.x > 10 or abs(p.position.y) > 10.0:
                continue

            tendency += p.position.y

        if print_tendency:
            if tendency > 13.0:
                rospy.loginfo("LEFT : %.4f" % tendency)
            elif tendency < -13.0:
                rospy.loginfo("RIGHT : %.4f" % tendency)
            else:
                rospy.loginfo("Tendency : %.4f" % tendency)

        self.tendency = tendency


    def obstacleCallback(self, msg):
        # Obstacle Callback. Automatically Calibrate and update self.obstacles
        self.current = rospy.Time.now()

        cones = []

        if self.test is True:
            for p in msg.poses:
                cone = [p.position.x, p.position.y]
                cones.append(cone)
                
            self.obstacles = cones
            
            return 0

        else:
            if self.tf_sub.canTransform(self.frame_id, "velodyne", rospy.Time(0)):
                for p in msg.poses:
                    dist = np.hypot(p.position.x, p.position.y)

                    if dist < 0.5 or dist > 15.0 or p.position.x < 0.0 or abs(p.position.y) > 10.0:
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

                if dist < 1.0:
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
        self.g_gain = 0.10
        self.threshold = 0

        self.err_stack = 0
        self.stack = 0

        self.loop_closure = False
        self.loop_closure_flag = False

        self.test_pub = rospy.Publisher(
            "/cone_tracker/node", PointStamped, queue_size=1)
        self.node = Node(data=[self.state.x, self.state.y], idx=0, parent=None)
        self.first_node = self.node
        self.last_idx = self.node.idx

        xrange = [-20, 40]
        yrange = [-40, 10]
        size = 0.25

        self.mapper = Mapper(xrange, yrange, size, True)

    def planning(self):
        
        if self.loop_closure is True:
            return createPathFromNode(self.node)
        
        else:
            if len(self.mapper.obstacles) < 4:
                rospy.logwarn("No Obstacle Datas")
                return None

            c_cost = d_cost = grid_cost = None

            min_cost = float("inf")
            node_dist = 0.0
            best_point = None

            points = self.getMiddlePoints(obstacles=self.mapper.obstacles)
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

                        if self.mapper.map.map[i][j].count <= 4:
                            grid_cost = self._calculateGridCost(self.node.data, point) * self.g_gain

                            total_cost = c_cost + d_cost + grid_cost
                            if total_cost <= min_cost:
                                min_cost = total_cost
                                best_point = point
            
            
            # print(d_cost, c_cost, grid_cost)
            
            if best_point is None or min_cost == float("inf"):
                rospy.logwarn("Cannot find best node...")
                self.stack += 1
                
                self.stack = np.clip(self.stack, 0, 2)
                
                for _ in range(self.stack):
                    self.node.rollback()
                
                path = createPathFromNode(self.node)
                
                return path
            
            
            else:
                self.stack -= 1
                
                if self.stack < 0:
                    self.stack = 0
                
                if self.node.parent is not None:
                    car_vec = np.array(
                        [m.cos(self.state.yaw), m.sin(self.state.yaw)])
                    point_vec = np.array([
                        self.node.data[0] - self.node.parent.data[0], self.node.data[1] - self.node.parent.data[1]
                    ])     
            
                    theta = abs(np.dot(car_vec, point_vec) / (np.hypot(point_vec[0], point_vec[1])))
                    if m.degrees(theta) > 150:
                        
                        for _ in range(10):
                            rospy.logfatal("Tlqkf")
                            self.node.rollback()
            
            
            new_node = Node(data=best_point, idx=(self.node.idx + 1), parent=self.node)
            
            dist = np.hypot(
                    new_node.data[0] - self.first_node.data[0],
                    new_node.data[1] - self.first_node.data[1]
                )
            
            if dist < 2.0 and self.last_idx > 50 and self.loop_closure is False and self.loop_closure_flag is False:
                self.loop_closure_flag = True
                self.node = Node(data=self.first_node.data, idx=(self.node.idx + 1), parent=self.node)
                
            elif self.loop_closure_flag is True:
                dist2 = np.hypot(self.first_node.data[0] - self.state.x, self.first_node.data[1] - self.state.y)
                if dist2 < 5.0:
                    self.loop_closure = True
            
            elif self.loop_closure is False and self.loop_closure_flag is False: 
                self.node = new_node

            if self.node.idx > self.last_idx:
                self.last_idx = self.node.idx

            self.test_pub.publish(self.node.parsePoint())

            path = createPathFromNode(self.node)

            return path

    def _calculateDistanceCost(self, point):
        distance = np.hypot(
            self.node.data[0] - point[0], self.node.data[1] - point[1])
        car_dist = np.hypot(
            self.state.x - point[0], self.state.y - point[1]
        )
        
        b_dist = (distance > 0.2 and distance < 10.0)
        b_car = (car_dist > 0.2 and car_dist < 10.0)
        
        return (b_dist and b_car), distance * self.d_gain

    def _calculateCurveCost(self, state_vec, point):
        
        car_vec = np.array([m.cos(self.state.yaw), m.sin(self.state.yaw), 0.0])
        state_vec = np.array([state_vec[0], state_vec[1], 0.0])
        point_vec = np.array([
            point[0] - self.node.data[0], point[1] - self.node.data[1], 0.0
        ])
        
        cost_gain = 0.33    # infinity range
        gain = self.c_gain  # cost gain
        
        d2 = 0.0
        
        
        try:
            if np.dot(car_vec, point_vec) < 0:
                return False, float("inf")
    
        
            if abs(self.mapper.tendency) >= 13.0:
                target_vec = np.array([point[0] - self.state.x, point[1] - self.state.y, 0.0])
                up_vec = np.array([0., 0., 1.0])
                
                cross_vec = np.cross(target_vec, car_vec)
                d2 = -np.dot(cross_vec, up_vec)
                            
                
            dot = float(np.dot(state_vec, point_vec))
            det_s = float(np.linalg.norm(state_vec))
            det_p = float(np.linalg.norm(point_vec))
            
            if det_s == 0 or det_p == 0:
                return False, float("inf")
            
            theta = abs(m.acos(dot / (det_s * det_p)))
                
            if (abs(self.mapper.tendency) >= 13.0): 
                theta = abs(theta - m.radians(abs(self.mapper.tendency) / 2))
                
                if (d2 < 0 and self.mapper.tendency < -13.0) or (d2 > 0 and self.mapper.tendency > 13.0):
                    cost_gain = 0.7
                    gain *= 0.8
                    
                elif (d2 < 0 and self.mapper.tendency > 13.0) or (d2 > 0 and self.mapper.tendency < -13.0):
                    gain *= float("inf")
                    

        except Exception as ex:
            rospy.logwarn("WTF")
            rospy.logwarn(ex)
            return False, float("inf")

        cost = theta * gain

        if theta >= (m.pi / 2.0) * cost_gain:
            cost = float("inf")

        return dot > 0, cost

    def _calculateGridCost(self, last, point):
        total_cost = 0

        si, sj = self.mapper.map.getGridIdx(last)
        gi, gj = self.mapper.map.getGridIdx(point)
        
        # c = self.mapper.map.map[gi][gj].count
        # if c == 0:
        #     return float("inf")

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
                if cost >= 7:
                    return float("inf")
                total_cost += cost

            for j in range(d):
                cost = self.mapper.map.map[si +
                                           (i * (1 if positive_x else -1)) + (j * (1 if positive_x else -1))][sj + (i * (1 if positive_y else -1))].count
                if cost >= 7:
                    return float("inf")
                total_cost += cost

        else:
            for i in range(abs(dist_x)):
                cost = self.mapper.map.map[si +
                                           (i * (1 if positive_x else -1))][sj + (i * (1 if positive_y else -1))].count
                if cost >= 7:
                    return float("inf")
                total_cost += cost

            for j in range(d):
                cost = self.mapper.map.map[si +
                                           (i * (1 if positive_x else -1))][sj + (i * (1 if positive_y else -1)) + (j * (1 if positive_y else -1))].count
                if cost >= 7:
                    return float("inf")
                total_cost += cost

        return total_cost

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
    def __init__(self, data, idx=0, parent=None):
        self.data = data
        self.idx = idx
        self.parent = parent


    def findBestNode(self, map):
        # map = GridMap()
        i, j = map.getGridIdx(self.data)

        best_grid = map.map[i][j]
        best_cost = best_grid.count
        
        pi = pj = 0
        
        if best_cost < 2:
            return i, j
        

        for a in range(-1, 2):
            for b in range(-1, 2):
                grid = map.map[i + a][j + b]
                new_cost = grid.count
                if new_cost < best_cost and new_cost != 0:
                    best_grid = grid
                    best_cost = new_cost
                    pi = i + a
                    pj = j + b

        if self.parent is not None:
            if self.parent.data[0] == best_grid.x and self.parent.data[1] == best_grid.y:
                return i, j

        self.data = [best_grid.x, best_grid.y]
        return pi, pj

    def rollback(self):
        if self.parent is not None:
            self.data = self.parent.data
            self.idx = self.parent.idx
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

    last_i = last_j = 0
    
    t = True

    while True:
        if node is None:
            break
        
        if t is True:
            xs.append(node.data[0])
            ys.append(node.data[1])

        else:
            i, j = node.findBestNode(planner.mapper.map)

            if abs(i - last_i) + abs(j - last_j) != 0:
                xs.append(node.data[0])
                ys.append(node.data[1])
                
                last_i = i
                last_j = j

        node = node.parent


    try:
        cx, cy, cyaw, _, _ = calc_spline_course(
            list(reversed(xs)), list(reversed(ys)), 0.1)

        return PathResponse(None, None, None, cx, cy, cyaw)

    except Exception as ex:
        rospy.logwarn(ex)
        pass

    return None

def wait_for_stop(duration, brake=60):
    global current_time, last_time, r, cmd_pub

    msg = ControlMessage(0, 0, 2, 0, 0, brake, 0)

    dt = 0
    last_time = rospy.Time.now()
    while dt < duration:
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()

        if dt > duration:
            last_time = current_time

        cmd_pub.publish(msg)
        r.sleep()
        
        

class PurePursuit(object):
    def __init__(self, state):
        super(PurePursuit, self).__init__()

        """ Input Var """

        self.state = state

        self.delta_ref = 0.0  # steer
        
        self.servo_bias = 0.0  # [rad] 0.03

        self.v = 3.0
        self.K = rospy.get_param("k_gain", 1.0)
        self.L = 1.040

        self.Lr = self.v * self.K - self.L
        self.Ld = 0.0

    def set_steering(self, goal):  # 5 calculate the new required steering angle
        dx = goal[0] - self.state.x
        dy = goal[1] - self.state.y

        if dx == 0.0:
            self.delta_ref = 0.0
            return self.delta_ref

        self.Ld = np.hypot(dx, dy)
        alpha = m.atan(dy / dx) - self.state.yaw

        if dx > 0:
            alpha *= -1.0

        self.delta_ref = m.atan(2 * self.L * m.sin(alpha) / self.Ld)

        return self.delta_ref
    

if __name__ == "__main__":
    rospy.init_node("cone_tracker")
    
    last_time = rospy.Time.now()
    current_time = rospy.Time.now()

    state = State(odometry_topic="/odometry/kalman", hz=10, test=True)
    stanley = Stanley()
    pp = PurePursuit(state)
    
    stanley.setCGain(0.25)
    
    pid = PID()

    sleep(3.0)

    planner = PathPlanner(state=state)

    grid_pub = rospy.Publisher(
        "/cone_tracker/gridmap", OccupancyGrid, queue_size=1)
    
    target_pub = rospy.Publisher(
        "/cone_tracker/target_point", PointStamped, queue_size=1
    )
    
    path_pub = rospy.Publisher("/cone_tracker/cone_path", Path, queue_size=10)

    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)

    path = None
    target_idx = 0


    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        t1 = rospy.Time.now()
        
        
        msg = ControlMessage()
        msg.Gear = int(2)
        
        new_layer = GridMap(planner.mapper.xrange, planner.mapper.yrange,
                            planner.mapper.size, obstacles=planner.mapper.obstacles)
        planner.mapper.map = new_layer
        # planner.mapper.map.addObstacles(planner.mapper.obstacles)

        if grid_pub.get_num_connections() > 0:
            grid_pub.publish(planner.mapper.map.parseOccupiedGrid())

        temp = planner.planning()

        if temp is not None:
            path = temp

        try:
            if path is not None:    
                
                if planner.loop_closure is True:
                    if target_idx >= len(path.cx) - 20:
                        target_idx = 0
                
                current_i, current_j = planner.mapper.map.getGridIdx(point=[state.x, state.y])
                
                target_idx, _ = stanley.calc_target_index(state, path.cx, path.cy, False)
                
                ip = 0
                
                irange = range(0, 0)
                jrange = range(0, 0)
                
                
                if len(path.cx) - 20 < target_idx:
                    m_target_idx = len(path.cx) - 5
                else:
                    m_target_idx = target_idx + 15   
                               
                while True:
                    # print(target_idx, m_target_idx)
                    
                    target_i, target_j = planner.mapper.map.getGridIdx(point=[path.cx[m_target_idx + ip], path.cy[m_target_idx + ip]])
                    direct_i = target_i - current_i
                    direct_j = target_j - current_j
                    
                    if direct_i == 0 and direct_j == 0:
                        ip += 1
                        
                    else:
                        if target_i == 0:
                            jrange = range(-2, 3)
                        
                        elif target_j == 0:
                            irange = range(-2, 3)
                            pass
                        
                        else:
                            if direct_i > 0:
                                irange = range(0, 3)
                            else:
                                irange = range(-2 ,1)
                            
                            if direct_j > 0:
                                jrange = range(0, 3)
                            else:
                                jrange = range(-2, 1)
                        
                        
                        m_best_grid = planner.mapper.map.map[target_i][target_j]
                        t_cost = m_best_grid.count
                        
                        for a in irange:
                            for b in jrange:
                                m_grid = planner.mapper.map.map[target_i + a][target_j + b]
                                m_new_cost = m_grid.count
                                if m_new_cost < t_cost:
                                    m_best_grid = m_grid
                                    t_cost = m_new_cost
                        
                        di = pp.set_steering([m_best_grid.x, m_best_grid.y])
                        
                        break
                
                # di, target_idx = stanley.stanley_control(
                #     state, path.cx, path.cy, path.cyaw, last_target_idx=target_idx)
                
                di = np.clip(di, -m.radians(30), m.radians(30))
                
                
                target_speed = 5 if (planner.loop_closure is False) else 7
                dl = (len(path.cx) - 1) - target_idx
                
                if dl < 40:
                    target_speed = 3
                
                elif dl < 20:
                    target_speed = 0


                target_speed *= (1 - abs(di) * 0.6)

                p = PointStamped(
                    Header(None, rospy.Time.now(), frame_id),
                    Point(m_best_grid.x, m_best_grid.y, 0.0)
                )
                
                
                pid_speed = pid.PIDControl(target_speed)
                msg.Speed = int(pid_speed)
                msg.Steer = int(m.degrees(di))

                target_pub.publish(p)
                path_pub.publish(planner.parsePath(path))

            else:
                rospy.logwarn("NO PATH")


        except IndexError as ie:
            rospy.logfatal(ie)
            target_idx = 0

        except Exception as ex:
            rospy.logfatal(ex)
            rospy.logwarn("Cannot make cmd_msg")
            msg.Speed = int(0)
            msg.Steer = int(0)

        cmd_pub.publish(msg)
        
        
        t2 = rospy.Time.now()
        
        # rospy.loginfo("TIME : %.6f" % (t2 - t1).to_sec())

        r.sleep()
