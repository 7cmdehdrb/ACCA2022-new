#!/usr/bin/env python

import rospy
import rospkg
import tf
import csv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# -21.4563045502,-46.2504959106,-1.29778019985,1.904,3.434
# -21.7034759521,-48.9654655457,-1.37349931797,1.904,3.434


"""

Export module

Read csv which is located in /path_planner/saved_path/(FILE NAME) and
Insert data into self.cx, self.cy, self.cyaw

"""


class LoadPose(object):
    def __init__(self, file_name="static_path.csv"):
        super(LoadPose, self).__init__()

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.file_name = file_name

        self.readCSV()

    def readCSV(self):
        output_file_path = rospkg.RosPack().get_path('control') + \
            "/saved_path/" + str(self.file_name)

        text = "LOADING <"
        text += str(self.file_name)
        text += "> ......"

        cnt = 0

        rospy.loginfo(text)
        with open(output_file_path, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")
            for row in reader:
                self.cx.append(float(row[0]))
                self.cy.append(float(row[1]))
                self.cyaw.append(float(row[2]))
                cnt += 1

        rospy.loginfo("LOADING FINISHED")
        rospy.loginfo("LOAD " + str(cnt) + " LINE")
        rospy.loginfo("")

    def pathPublish(self, pub):
        msg = Path()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.poses = []

        # use planner, create cx, cy, cyaw

        for i in range(0, len(self.cx)):
            pose = PoseStamped()

            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"

            quat = tf.transformations.quaternion_from_euler(0, 0, self.cyaw[i])

            pose.pose.position.x = self.cx[i]
            pose.pose.position.y = self.cy[i]
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            msg.poses.append(pose)

        # for i in range(5):
        pub.publish(msg)
