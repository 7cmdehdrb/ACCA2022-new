#!/usr/bin/env python

import sys
import os
import rospy
import rospkg
import numpy as np
import math as m
import csv
import genpy
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Empty, String


class WayPoint(object):
    def __init__(self, id, pose=Point(), is_end=False):
        self.id = id
        self.pose = pose
        self.is_end = is_end

    def parseId(self):
        area = self.id[0]
        idx = self.id[1]

        return int(str(ord(area)) + idx)

    def parseMarker(self):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = self.id
        marker.id = self.parseId()

        marker.text = self.id

        marker.type = 9
        marker.action = 0

        marker.pose.position = self.pose
        marker.pose.orientation = Quaternion(0., 0., 0., 1.)
        marker.scale = Vector3(5., 5., 5.)
        marker.color = ColorRGBA(
            1., 1., 1., 1.) if self.is_end is False else ColorRGBA(1., 0., 0., 1.)

        marker.lifetime = genpy.Duration(secs=1)

        return marker


class TemperalPoint(object):
    def __init__(self):
        self.pub = rospy.Publisher("/temperal_point", Marker, queue_size=1)
        self.sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, callback=self.temperalPointCallback)

        self.do_save = rospy.get_param("/waypoints/save_waypoint", True)

        self.marker = Marker()

    def publishMarker(self):
        self.pub.publish(self.marker)

        return 0

    def deleteMarker(self):
        self.marker.action = 2
        self.publishMarker()

        return 0

    def temperalPointCallback(self, msg):

        if self.do_save is False:
            return 0

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "temp"
        marker.id = 0

        marker.type = 9
        marker.action = 0

        marker.pose = msg.pose
        marker.scale = Vector3(5., 5., 5.)
        marker.color = ColorRGBA(1., 1., 1., 1.)

        marker.text = "HERE!!"

        self.marker = marker

        self.publishMarker()

        return 0


class WayPoints(object):
    def __init__(self):
        self.file_name = rospy.get_param(
            "/waypoints/waypoints_file", "path.csv")
        self.path = rospkg.RosPack().get_path("path_plan") + \
            "/waypoints/" + self.file_name

        self.__waypoints = []
        self.__waypoints_pub = rospy.Publisher(
            "waypoints", MarkerArray, queue_size=1)
        self.__add_waypoint_sub = rospy.Subscriber(
            "/add_waypoint", String, self.addWaypoint)
        self.__delete_waypoint_sub = rospy.Subscriber(
            "/delete_waypoint", String, self.deleteWaypoint)
        self.__save_waypoints_sub = rospy.Subscriber(
            "/save_waypoints", Empty, callback=self.saveWaypoints)

        self.temperal_point = TemperalPoint()

        self.loadWaypoints()

    def saveWaypoints(self, msg):
        with open(self.path, 'w') as csvfile:
            for wp in self.__waypoints:
                try:
                    id = wp.id  # string
                    x = str(wp.pose.x)   # float
                    y = str(wp.pose.y)   # float

                    csvfile.write(id + "," + x + "," + y + "\n")

                except Exception as ex:
                    rospy.logwarn(ex)

        return True

    # Initial function. Read CSV
    def loadWaypoints(self):
        try:
            with open(self.path, "r") as csvFile:
                reader = csv.reader(csvFile, delimiter=",")
                for row in reader:
                    waypoint = WayPoint(id=row[0], pose=Point(
                        float(row[1]), float(row[2]), 0.), is_end=("1" == row[3]))
                    self.__waypoints.append(waypoint)
        except Exception as ex:
            rospy.logwarn(ex)
            os.system(
                "touch touch ~/catkin_ws/src/ACCA2022-new/erp42/erp42_control/waypoints/%s" % self.file_name
            )

        self.publishWaypoints()

    def checkDuplicate(self, waypoint):
        for i in range(len(self.__waypoints)):
            if waypoint.id == self.__waypoints[i].id:
                return i
        return -1

    def addWaypoint(self, msg):

        id, is_end = msg.data.split("/")

        waypoint = WayPoint(
            id=id, pose=self.temperal_point.marker.pose.position, is_end=("1" == is_end))

        if self.checkDuplicate(waypoint) == -1:
            self.__waypoints.append(waypoint)

            self.publishWaypoints()
            self.temperal_point.deleteMarker()

        else:
            rospy.logwarn("Waypoint " + waypoint.id +
                          " is already existed!!")

    def deleteWaypoint(self, msg):
        id = msg.data

        rospy.logwarn("DELETE WAYPOINT...")

        waypoint = WayPoint(id, Point())

        idx = self.checkDuplicate(waypoint)
        if idx != -1:
            rospy.loginfo("SUCCESS...")
            del self.__waypoints[idx]
            return True

        rospy.logwarn("FAIL")

        return False

    def publishWaypoints(self):
        msg = MarkerArray()

        for wp in self.__waypoints:
            msg.markers.append(wp.parseMarker())

        self.__waypoints_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("waypoints")

    node = WayPoints()

    r = rospy.Rate(3)
    while not rospy.is_shutdown():
        node.publishWaypoints()
        r.sleep()
