#!/usr/bin/env python

from yaml import load
import rospy
import time
import csv
from time import sleep
from DB import *
from path_plan.msg import PathRequest, PathResponse
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

db_name = rospy.get_param("/LoadPath/db_name", "/path.db")


class LoadPath():
    def __init__(self, db):
        self.db = db
        self.Request = PathRequest()
        self.trig = False

    def RequestCallback(self, msg):
        rospy.loginfo("Request Accepted!")
        self.Request = msg
        self.trig = True

    def bringPath(self):
        # bring path
        self.path_id = self.db.bring_path_id(
            self.Request.start, self.Request.end)
        self.path_info = self.db.bring_pathinfo(self.path_id)
        # [[x, y, yaw]]

    def listToPath(self):
        # PathResponse publish path
        Response = PathResponse()

        Response.start = self.Request.start
        Response.end = self.Request.end
        Response.path_id = self.Request.path_id

        for info in self.path_info:
            Response.cx.append(info[0])
            Response.cy.append(info[1])
            Response.cyaw.append(info[2])

        return Response

    def toRosPath(self):
        # ros path publish
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for info in self.path_info:

            pose = PoseStamped()

            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = info[0]
            pose.pose.position.y = info[1]

            quat = quaternion_from_euler(0., 0., info[2])

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            path.poses.append(pose)

        return path

    def check_path_avaliable(self):
        path = []
        file_path = rospkg.RosPack().get_path("path_plan") + \
            "/path/" + rospy.get_param("/LoadPath/path_name", "path.csv")

        with open(file_path, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")
            for row in reader:
                try:
                    self.Request.start = row[0]
                    self.Request.end = row[1]
                    self.bringPath()
                    for info in self.path_info:
                        path.append(info)

                except ValueError as ex:
                    rospy.logwarn(ex)

                except IndexError as ie:
                    # os.system("killall -9 rosmaster")
                    os.system("rosnode kill --all")
                    rospy.logfatal("No path data")
                    rospy.logfatal(ie)
                    raise Exception()

            self.path_info = path
            ros_path = self.toRosPath()

            for i in range(10):
                rospath_pub.publish(ros_path)
                sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("LoadPath")

    db = DB(db_name)
    load_path = LoadPath(db)

    PathPoint_sub = rospy.Subscriber(
        "/path_request", PathRequest, callback=load_path.RequestCallback)
    listpath_pub = rospy.Publisher(
        "/path_response", PathResponse, queue_size=1)
    rospath_pub = rospy.Publisher("/global_path", Path, queue_size=1)

    load_path.check_path_avaliable()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        if load_path.trig is True:

            try:
                load_path.bringPath()
                res = load_path.listToPath()
                listpath_pub.publish(res)

                if PathPoint_sub.get_num_connections() > 0:
                    ros_path = load_path.toRosPath()
                    rospath_pub.publish(ros_path)
                    rospy.loginfo('ros path published')

            except Exception as ex:
                rospy.logwarn(ex)

            finally:
                load_path.trig = False

        r.sleep()
