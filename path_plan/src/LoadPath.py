#!/usr/bin/env python

import rospy
import time
from time import sleep
from DB import *
from path_plan.msg import PathRequest, PathResponse
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


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
        self.path = Path()
        self.path.header.frame_id = "map"
        self.path.header.stamp = rospy.Time.now()

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

            self.path.poses.append(pose)


if __name__ == "__main__":
    rospy.init_node("LoadPath")

    db = DB()
    load_path = LoadPath(db)

    PathPoint_sub = rospy.Subscriber(
        "/path_request", PathRequest, callback=load_path.RequestCallback)
    listpath_pub = rospy.Publisher(
        "/path_response", PathResponse, queue_size=1)
    rospath_pub = rospy.Publisher("/global_path", Path, queue_size=1)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if load_path.trig is True:

            try:
                load_path.bringPath()
                res = load_path.listToPath()
                listpath_pub.publish(res)

                if PathPoint_sub.get_num_connections() > 0:
                    load_path.toRosPath()
                    rospath_pub.publish(load_path.path)
                    rospy.loginfo('ros path published')

            except Exception as ex:
                rospy.logwarn(ex)

            finally:
                load_path.trig = False

        r.sleep()
