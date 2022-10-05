#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import math as m
import csv
from enum import Enum
from path_plan.msg import PathRequest, PathResponse


class PathType(Enum):
    STRAIGHT = 0
    RIGHT = 1
    LEFT = 2
    NONE = 3
    UTURN = 4


class MissionState(Enum):
    DRIVING = 0
    TRAFFIC = 1
    DELIVERY_A = 2
    DELIVERY_B = 3
    STATIC = 4
    DYNAMIC = 5
    PARKING = 6
    RIGHT = 7
    END = 9


class Waypoint(object):
    def __init__(self, id, is_end):
        self.id = id
        self.is_end = is_end
        print(id, is_end)


def findWaypoint(file_path, id):
    with open(file_path, "r") as csvFile:
        reader = csv.reader(csvFile, delimiter=",")
        for row in reader:
            if id == row[0]:
                return Waypoint(id=row[0], is_end=("1" == row[3]))


class Node(object):
    def __init__(self, data, start, end, next=None, desired_speed=0, path_type=PathType.NONE, mission_type=MissionState.DRIVING):
        self.data = data
        self.desired_speed = desired_speed
        self.path_type = path_type
        self.mission_type = mission_type
        self.next = next

        self.start = start
        self.end = end
        self.type = None

    def append(self, data):
        self.next = data


class PathSelector(object):
    def __init__(self, state):
        self.state = state

        self.req_pub = rospy.Publisher(
            "/path_request", PathRequest, queue_size=1)

        # Get All Path Data
        self.path = None
        self.getAllPath()

    def goNext(self):
        if self.path.next is not None:
            self.path = self.path.next
            rospy.loginfo("Change to next path!")
            rospy.loginfo("%s - %s" % (self.path.start.id, self.path.end.id))
            return self.path

        return None

    def makeRequest(self):
        if self.path is None:
            rospy.logwarn("Loading Path Data...")
            return 0

        # rospy.loginfo("Send Request...")
        rospy.loginfo(self.path.start)

        self.req_pub.publish(self.path.data)

    def getAllPath(self):
        waypoints_path = rospkg.RosPack().get_path("path_plan") + "/waypoints/" + \
            rospy.get_param("/waypoints/waypoints_file", "waypoints.csv")
        file_path = rospkg.RosPack().get_path("path_plan") + "/path/" + \
            rospy.get_param("/LoadPath/path_name", "festi_path.csv")

        node = None

        with open(file_path, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")
            for row in reader:
                start = row[0]
                end = row[1]
                desired_speed = float(row[2])
                path_type = PathType(int(row[3]))
                mission_type = MissionState(int(row[4]))

                start_point = findWaypoint(waypoints_path, id=start)
                end_point = findWaypoint(waypoints_path, id=end)

                temp = Node(PathRequest(start, end, start+end),
                            start=start_point, end=end_point, desired_speed=desired_speed, path_type=path_type, mission_type=mission_type)

                if node is None:
                    node = temp
                    self.path = node

                else:
                    node.append(temp)
                    node = temp

                    if len(row) == 6:
                        self.path = node

        return 0
