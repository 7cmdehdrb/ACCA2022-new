#!/usr/bin/env python

import os
import sys
import rospy
import rospkg
import numpy as np
import math as m
from enum import Enum
from random import randint, random
import genpy
import tf
from tf.transformations import *

# msg
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from erp42_control.msg import *

# custum
from cone_tracker import Cone


try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)


class Simulator(object):
    def __init__(self):
        self.cones = self.createVirtualCones()
        self.state = State(odometry_topic="/odometry/kalman", hz=30, test=True)

        self.cones_pub = rospy.Publisher(
            "/cone_simulator/cones", PoseArray, queue_size=1)

    def createVirtualCones(self):
        temp = []
        j = 0

        for i in range(0, 120):
            cone1 = Cone(
                x=i + randint(-10, 10) * 0.01,
                y=j - (2 + randint(0, 10) * 0.05)
            )

            cone2 = Cone(
                x=i + randint(-10, 10) * 0.01,
                y=j + (2 + randint(0, 10) * 0.05)
            )

            temp.append(cone1)
            temp.append(cone2)

            j = 16 * (m.sin(i * 0.1)) ** 3

        return temp

    def publishAllCones(self):
        msg = PoseArray()

        msg.header = Header(None, rospy.Time.now(), "map")
        for cone in self.cones:
            p = Pose()
            p.position.x = cone.x
            p.position.y = cone.y
            msg.poses.append(p)

        self.cones_pub.publish(msg)

    def detectCones(self):
        msg = PoseArray()

        msg.header = Header(None, rospy.Time.now(), "map")

        for cone in self.cones:
            dist = np.hypot(cone.x - self.state.x, cone.y - self.state.y)
            if dist < 5.0:
                p = Pose()
                p.position.x = cone.x + (randint(-100, 100) * 0.001)
                p.position.y = cone.y + (randint(-100, 100) * 0.001)

                msg.poses.append(p)

        return msg


if __name__ == "__main__":
    rospy.init_node("cone_simulator")

    simulator = Simulator()

    detected_cones_pub = rospy.Publisher(
        "/cone_simulator/detected_cones", PoseArray, queue_size=1)

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        detected_cones_pub.publish(simulator.detectCones())
        simulator.publishAllCones()

        r.sleep()
