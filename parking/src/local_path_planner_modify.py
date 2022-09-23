#!/usr/bin/env python
# -*- coding: utf-8 -*-


from time import sleep
import rospy
import rospkg
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import *
from cubic_spline_planner import calc_spline_course
# from dynamic_window_approach import *
from parking_area import ParkingArea
from rrt_star_reeds_shepp import *
from std_msgs.msg import Int8
from path_plan.msg import PathResponse

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State
except Exception as ex:
    rospy.logfatal(ex)


class Local_path_planner():
    def __init__(self):
        self.path = PathResponse()
        self.path.cx = 0
        self.path.cy = 0
        self.path.cyaw = 0
        self.scale_x = 2.5
        self.scale_y = 5.0
        self.yaw = 0.4
        self.parking_areas = []
        self.points_of_parking_areas = []
        self.center_points = []
        self.target_area_Idx = 0

    def toRosPath(self, xs, ys, yaws):
        # ros path publish
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for i in range(len(xs)):

            pose = PoseStamped()

            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = xs[i]
            pose.pose.position.y = ys[i]

            quat = quaternion_from_euler(0., 0., yaws[i])

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            path.poses.append(pose)

        rospath_pub.publish(path)

    def publishPath(self, pub, cx, cy, cyaw):

        path = PathResponse()

        path.cx = cx
        path.cy = cy
        path.cyaw = cyaw

        pub.publish(path)

        return path

    def erase_other_zone(self):

        obstacleList = []
        x_interval = self.center_points[1][0] - \
            self.center_points[0][0]

        for i in [0, 5]:
            sign = - (-1)**i
            xc = self.center_points[i][0] + \
                sign * x_interval  # 중심점의 x좌표
            yc = self.center_points[i][1]  # 중심점의 y좌표

            x1 = xc + self.scale_x / 2 * m.cos(self.yaw)
            y1 = yc + self.scale_x / 2 * m.sin(self.yaw)

            x2 = xc - self.scale_x / 2 * m.cos(self.yaw)
            y2 = yc - self.scale_x / 2 * m.sin(self.yaw)

            o_c = [xc, yc, self.scale_x/2]
            o_1 = [x1, y1, self.scale_x/2]
            o_2 = [x2, y2, self.scale_x/2]

            obstacleList.append(o_c)
            obstacleList.append(o_1)
            obstacleList.append(o_2)

        for i in range(self.the_number_of_parkinarea):

            if i != self.target_area_Idx:
                xc = self.center_points[i][0]  # 중심점의 x좌표
                yc = self.center_points[i][1]  # 중심점의 y좌표

                x1 = xc + self.scale_x / 2 * m.cos(self.yaw)
                y1 = yc + self.scale_y / 2 * m.sin(self.yaw)

                x2 = xc - self.scale_x / 2 * m.cos(self.yaw)
                y2 = yc - self.scale_y / 2 * m.sin(self.yaw)

                o_c = [xc, yc, self.scale_x/2]
                o_1 = [x1, y1, self.scale_x/2]
                o_2 = [x2, y2, self.scale_x/2]

                obstacleList.append(o_c)
                obstacleList.append(o_1)
                obstacleList.append(o_2)

        return obstacleList

    def start_callback(self, msg):
        self.start_x = msg.position.x
        self.start_y = msg.position.y
        _, _, self.start_yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        rospy.loginfo('start point is received')
        start_pose_sub.unregister()

    def markerCallback(self, msg):

        for marker in msg.markers:
            point = marker.pose.position
            orientation = marker.pose.orientation
            scale = marker.scale
            self.scale_x = scale.x
            self.scale_y = scale.y

            self.parking_area = ParkingArea(
                x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x)
            self.parking_areas.append(ParkingArea(
                x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x))

            self.points_of_parking_areas.append(
                self.parking_area.parseArray().tolist())

            center_point = [point.x, point.y]  # 주차 라인 중앙 점 좌표
            self.center_points.append(
                center_point)  # 주차 라인 중앙 점 좌표 list
            self.the_number_of_parkinarea = len(self.center_points)
        _, _, self.yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        rospy.loginfo("Subscribe MarkerArray")

        marker_sub.unregister()

    def parking_zone_callback(self, msg):
        self.target_area_Idx = msg.data
        print(self.target_area_Idx)

    def WP3_callback(self, msg):
        self.WP3_x = msg.x
        self.WP3_y = msg.y


if __name__ == "__main__":
    rospy.init_node("parking_local_path_planner")

    local_path_planner = Local_path_planner()

    start_pose_sub = rospy.Subscriber(
        "/startpose", Pose, callback=local_path_planner.start_callback)

    marker_sub = rospy.Subscriber(
        "/parking_areas", MarkerArray, callback=local_path_planner.markerCallback)

    # tate = State("/odometry/kalman")

    rospath_pub = rospy.Publisher("/local_path", Path, queue_size=1)

    parking_zone_sub = rospy.Subscriber(
        '/target_zone', Int8, callback=local_path_planner.parking_zone_callback)

    obstacle_pub = rospy.Publisher(
        "/obstacles", PoseArray, queue_size=1
    )
    path_pub = rospy.Publisher(
        "/path", PathResponse, queue_size=1
    )

    WP3_sub = rospy.Subscriber(
        '/parking_WP3',  Point, callback=local_path_planner.WP3_callback)
    sleep(1.)

    rospy.wait_for_message('/target_zone', Int8)

    '''obstacleList = []'''  # [x,y,size(radius)]
    target_idx = local_path_planner.target_area_Idx - 1

    print("@@@@@@@@@@@")
    obstacleList = local_path_planner.erase_other_zone()
    print("@@@@@@@@@@")
    print(obstacleList)

    '''for i, parking in enumerate(parking_areas):
        if i != target_idx:
            for j in parking.parseArray():
                # [x, y]
                obstacleList.append((j[0], j[1], 0.5))'''

    local_path_former_part_x, local_path_former_part_y, local_path_former_part_yaw, _, _ = calc_spline_course(
        [local_path_planner.start_x, local_path_planner.WP3_x], [local_path_planner.start_y, local_path_planner.WP3_y])

    start = [local_path_planner.WP3_x,
             local_path_planner.WP3_y, local_path_former_part_yaw[-1]]
    goal = [local_path_planner.parking_areas[target_idx].position.x,
            local_path_planner.parking_areas[target_idx].position.y, local_path_planner.yaw]

    msg = PoseArray()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()

    for parking in obstacleList:
        '''for i in parking.parseArray():'''
        x = parking[0]
        y = parking[1]

        new_pose = Pose()

        new_pose.position = Point(x, y, 0.)
        new_pose.orientation = Quaternion(0., 0., 0., 1.)

        msg.poses.append(new_pose)

    obstacle_pub.publish(msg)

    rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                             obstacleList,
                                             [0.0, 30.0], max_iter=100)
    print(start)
    print(goal)
    print(obstacleList)
    path = rrt_star_reeds_shepp.planning(animation=False)
    xs = [0]
    while len(xs) < 5:
        try:
            xs = [x for (x, y, syaw) in path]
            ys = [y for (x, y, syaw) in path]
            yaws = [syaw for (x, y, syaw) in path]
        except:
            pass

    local_path_planner.path.cx = local_path_former_part_x + xs
    local_path_planner.path.cy = local_path_former_part_y + ys
    local_path_planner.path.cyaw = local_path_former_part_yaw + yaws

    hz = 1.
    freq = 1 / hz

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        local_path_planner.publishPath(
            path_pub, local_path_planner.path.cx, local_path_planner.path.cy, local_path_planner.path.cyaw)
        local_path_planner.toRosPath(
            local_path_planner.path.cx, local_path_planner.path.cy, local_path_planner.path.cyaw)

        # rrt_star_reeds_shepp.draw_graph()
        # plt.plot([x for (x, y, syaw) in path], [
        #          y for (x, y, yaw) in path], '-r')
        # plt.grid(True)
        # plt.pause(0.001)
        # plt.show()

        r.sleep()

    rospy.spin()
