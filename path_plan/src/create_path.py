#!/usr/bin/env python

import rospy
import threading
import tf
import math as m
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from cubic_spline_planner import *


"""

Global path generator with RVIZ
Use '2D Nav Goal' to select waypoint

#####################################
HOW TO SAVE
#####################################
$ rostopic pub /save_path std_msgs/Empty -1

This command will save path data on /path_planner/saved_path/path.csv
(If path.csv file already exists, the path data will be overrided)

#####################################
HOW TO MODIFY PATH
#####################################
$ rostopic pub /delete_path std_msgs/Empty -1

This command will delete latest waypoint
After use this command, please wait for 5 seconds

"""


class GetPose(object):
    def __init__(self):
        super(GetPose, self).__init__()

        rospy.Subscriber(
            "/reset_path", Empty, self.resetCallback)

        self.deleteFlag = True

        self.initial_xs = []
        self.initial_ys = []

        self.Path = PoseArray()

        self.Path.header.seq = 0
        self.Path.header.frame_id = "map"

    def resetCallback(self, msg):
        self.initial_xs = []
        self.initial_ys = []

    def poseCallback(self, msg):
        """ 
            Subscribe initial pose
            add to initials 
        """

        temp_x = msg.pose.position.x
        temp_y = msg.pose.position.y

        self.initial_xs.append(temp_x)
        self.initial_ys.append(temp_y)

        rospy.loginfo("ADD POINT")

    def posePublish(self):
        msg = PoseArray()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        temp_poses = []

        # use planner, create cx, cy, cyaw

        cx = []
        cy = []
        cyaw = []

        try:
            cx, cy, cyaw, ck, s = calc_spline_course(
                self.initial_xs, self.initial_ys, ds=0.1)
        except Exception as ex:
            pass

        for i in range(0, len(cx)):
            pose = Pose()

            quat = tf.transformations.quaternion_from_euler(0, 0, cyaw[i])

            pose.position.x = cx[i]
            pose.position.y = cy[i]
            pose.position.z = 0.0

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            temp_poses.append(pose)

        msg.poses = temp_poses

        path_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("create_global_path")

    get_pose = GetPose()

    path_pub = rospy.Publisher("create_global_path", PoseArray, queue_size=1)
    rospy.Subscriber(
        "/move_base_simple/goal", PoseStamped, get_pose.poseCallback)

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        get_pose.posePublish()
        r.sleep()
