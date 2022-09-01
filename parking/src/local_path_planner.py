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
from std_msgs.msg import Int16, Float32MultiArray
# from parking.msg import surround_obstacle
from path_plan.msg import PathResponse

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State
except Exception as ex:
    rospy.logfatal(ex)

the_number_of_parkinarea = 6


'''
def publishPath(pub, cx, cy, cyaw):
    path = Path()

    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()

    for i in range(len(cx)):
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        quat = quaternion_from_euler(0., 0., cyaw[i])

        pose.pose.position = Point(cx[i], cy[i], 0.)
        pose.pose.orientation = Quaternion(
            quat[0], quat[1], quat[2], quat[3])

        path.poses.append(pose)

    pub.publish(path)
    '''


def publishPath(pub, cx, cy, cyaw):

    path = PathResponse()

    path.cx = cx
    path.cy = cy
    path.cyaw = cyaw

    pub.publish(path)

    return PathResponse()


def erase_other_zone():

    obstacleList = []
    x_interval = list_of_center_point_list[1][0] - \
        list_of_center_point_list[0][0]

    for i in [0, 5]:
        pl_mi = - (-1)**i
        xc = list_of_center_point_list[i][0] + \
            pl_mi * x_interval  # 중심점의 x좌표
        yc = list_of_center_point_list[i][1]  # 중심점의 y좌표

        x1 = xc + scale_x / 2 * m.cos(yaw)
        y1 = yc + scale_y / 2 * m.sin(yaw)

        x2 = xc - scale_x / 2 * m.cos(yaw)
        y2 = yc - scale_y / 2 * m.sin(yaw)

        o_c = [xc, yc, scale_x/2]
        o_1 = [x1, y1, scale_x/2]
        o_2 = [x2, y2, scale_x/2]

        obstacleList.append(o_c)
        obstacleList.append(o_1)
        obstacleList.append(o_2)

    for i in range(the_number_of_parkinarea):

        if i != target_idx:
            xc = list_of_center_point_list[i][0]  # 중심점의 x좌표
            yc = list_of_center_point_list[i][1]  # 중심점의 y좌표

            x1 = xc + scale_x / 2 * m.cos(yaw)
            y1 = yc + scale_y / 2 * m.sin(yaw)

            x2 = xc - scale_x / 2 * m.cos(yaw)
            y2 = yc - scale_y / 2 * m.sin(yaw)

            o_c = [xc, yc, scale_x/2]
            o_1 = [x1, y1, scale_x/2]
            o_2 = [x2, y2, scale_x/2]

            obstacleList.append(o_c)
            obstacleList.append(o_1)
            obstacleList.append(o_2)

    return obstacleList


def markerCallback(msg):
    points_of_parking_areas = []
    list_of_center_point_list = []

    for marker in msg.markers:
        point = marker.pose.position
        orientation = marker.pose.orientation
        scale = marker.scale
        scale_y = scale.y
        scale_x = scale.x
        parking_areas.append(ParkingArea(
            x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x))

        Parking_area = ParkingArea(
            x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x)
        points_of_parking_areas.append(Parking_area.parseArray().tolist())
        center_point_list = [point.x, point.y]  # 주차 라인 중앙 점 좌표
        list_of_center_point_list.append(
            center_point_list)  # 주차 라인 중앙 점 좌표 list
    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    rospy.loginfo("Subscribe MarkerArray")

    marker_sub.unregister()


def parking_zone_callback(msg):
    target_area_Idx = msg.data
    #  print(target_area_Idx)


if __name__ == "__main__":

    'global variable initial setting'
    global target_area_Idx
    global parking_areas
    global scale_y
    global scale_x
    global points_of_parking_areas
    global yaw
    global list_of_center_point_list

    list_of_obstacle = []
    parking_areas = []
    list_of_center_point_list = []
    yaw, scale_x, scale_y = 0, 2.5, 5.0
    firstToNow_vec = [0, 0]
    fifthToNow_vec = [0, 0]
    target_area_Idx = 1

    rospy.init_node("parking_local_path_planner")

    rospy.wait_for_message("/target_zone", Int16)

    state = State("/odometry/kalman")
    parking_areas = []

    marker_sub = rospy.Subscriber(
        "/parking_areas", MarkerArray, callback=markerCallback)

    parking_zone_sub = rospy.Subscriber(
        '/target_zone', Int16, callback=parking_zone_callback)

    obstacle_pub = rospy.Publisher(
        "/obstacles", PoseArray, queue_size=1
    )
    path_pub = rospy.Publisher(
        "/path", PathResponse, queue_size=1
    )

    rospy.wait_for_message("/parking_areas", MarkerArray)

    sleep(1.)

    '''obstacleList = []'''  # [x,y,size(radius)]
    target_idx = target_area_Idx - 1

    print("@@@@@@@@@@@")
    obstacleList = erase_other_zone()
    print("@@@@@@@@@@")
    print(obstacleList)

    '''for i, parking in enumerate(parking_areas):
        if i != target_idx:
            for j in parking.parseArray():
                # [x, y]
                obstacleList.append((j[0], j[1], 0.5))'''

    start = [0.0, 18.0, np.deg2rad(0.0)]

    quat = parking_areas[target_idx].orientation
    _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    goal = [parking_areas[target_idx].position.x,
            parking_areas[target_idx].position.y, yaw]

    hz = 1.
    freq = 1 / hz

    r = rospy.Rate(hz)
    print(len(obstacleList))
    while not rospy.is_shutdown():
        print('it is running')

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
                                                 [0.0, 20.0], max_iter=100)
        path = rrt_star_reeds_shepp.planning(animation=False)
        try:
            xs = [x for (x, y, syaw) in path]
            ys = [y for (x, y, syaw) in path]
            yaws = [syaw for (x, y, syaw) in path]

            publishPath(path_pub, xs, ys, yaws)
        except:
            pass

        # rrt_star_reeds_shepp.draw_graph()
        # plt.plot([x for (x, y, syaw) in path], [
        #          y for (x, y, yaw) in path], '-r')
        # plt.grid(True)
        # plt.pause(0.001)
        # plt.show()

        r.sleep()

    rospy.spin()
