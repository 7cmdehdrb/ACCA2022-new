#!/usr/bin/env python


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
from dynamic_window_approach import *
from parking_area import ParkingArea
from rrt_star_reeds_shepp import *

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State
except Exception as ex:
    rospy.logfatal(ex)


def markerCallback(msg):
    global parking_areas

    for marker in msg.markers:
        point = marker.pose.position
        orientation = marker.pose.orientation
        scale = marker.scale

        parking_areas.append(ParkingArea(
            x=point.x, y=point.y, quat=orientation, w=scale.y, h=scale.x))

    rospy.loginfo("Subscribe MarkerArray")

    marker_sub.unregister()


if __name__ == "__main__":
    rospy.init_node("parking_local_path_planner")

    state = State("/odometry/kalman")
    parking_areas = []

    marker_sub = rospy.Subscriber(
        "/parking_areas", MarkerArray, callback=markerCallback)
    obstacle_pub = rospy.Publisher(
        "/obstacles", PoseArray, queue_size=1
    )
    path_pub = rospy.Publisher(
        "/path", Path, queue_size=1
    )
    test_pub = rospy.Publisher(
        "/test", PoseStamped, queue_size=1
    )

    rospy.wait_for_message("/parking_areas", MarkerArray)

    sleep(1.)

    obstacleList = []  # [x,y,size(radius)]

    for i in parking_areas[0].parseArray():
        # [x, y]
        obstacleList.append((i[0], i[1], 0.2))

    start = [0.0, 0.0, np.deg2rad(0.0)]

    quat = parking_areas[1].orientation
    _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    goal = [parking_areas[1].position.x, parking_areas[1].position.y, yaw]

    hz = 1.
    freq = 1 / hz

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        for parking in parking_areas:
            for i in parking.parseArray():
                x = i[0]
                y = i[1]

                new_pose = Pose()

                new_pose.position = Point(x, y, 0.)
                new_pose.orientation = Quaternion(0., 0., 0., 1.)

                msg.poses.append(new_pose)

        obstacle_pub.publish(msg)

        # rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
        #                                          obstacleList,
        #                                          [-2.0, 15.0], max_iter=100)
        # path = rrt_star_reeds_shepp.planning(animation=True)

        # rrt_star_reeds_shepp.draw_graph()
        # plt.plot([x for (x, y, syaw) in path], [
        #          y for (x, y, yaw) in path], '-r')
        # plt.grid(True)
        # plt.pause(0.001)
        # plt.show()

        # break

        r.sleep()

    rospy.spin()
