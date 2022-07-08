#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import math as m
import csv
from path_plan.msg import PathRequest, PathResponse


class Node(object):
    def __init__(self, data, next=None):
        self.data = data
        self.next = next

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
            return self.path

        return None

    def makeRequest(self):
        if self.path is None:
            rospy.logwarn("Loading Path Data...")
            return 0

        rospy.loginfo("Send Request...")
        rospy.loginfo(self.path.data)

        self.req_pub.publish(self.path.data)

    def getAllPath(self):
        file_path = rospkg.RosPack().get_path("erp42_control") + "/path/global_path.csv"

        node = None

        with open(file_path, "r") as csvFile:
            reader = csv.reader(csvFile, delimiter=",")
            for row in reader:
                try:
                    start = row[0]
                    end = row[1]

                    print(start, end)

                    temp = Node(PathRequest(start, end, start+end))

                    if node is None:
                        node = temp
                        self.path = node

                    else:
                        node.append(temp)
                        node = temp

                except Exception as ex:
                    rospy.logwarn(ex)

        return 0
