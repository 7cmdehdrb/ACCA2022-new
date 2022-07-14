#!/usr/bin/env python


from matplotlib import scale
import rospy
import rospkg
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from genpy import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA


class ParkingArea(object):
    def __init__(self, x, y, quat, w, h):

        self.position = Point(x, y, 0.)
        self.orientation = quat
        self.scale = Vector3(h, w, 0.2)

    def parseMarker(self, id, duration=1):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.pose.position = self.position
        marker.pose.orientation = self.orientation
        marker.scale = self.scale

        marker.color = ColorRGBA(1., 0., 0., 0.2)

        marker.type = 1
        marker.id = id
        marker.ns = str(id)
        marker.lifetime = Duration(secs=duration)

        return marker

    def parseArray(self):
        _, _, yaw = euler_from_quaternion(
            [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])

        p1 = [self.position.x, self.position.y]
        p2 = [self.position.x + m.cos(yaw) * (self.scale.x / 2.0) + m.sin(yaw) * (self.scale.y / 2.0),
              self.position.y + m.sin(yaw) * (self.scale.x / 2.0) - m.cos(yaw) * (self.scale.y / 2.0)]
        p3 = [self.position.x - m.cos(yaw) * (self.scale.x / 2.0) - m.sin(yaw) * (self.scale.y / 2.0),
              self.position.y - m.sin(yaw) * (self.scale.x / 2.0) + m.cos(yaw) * (self.scale.y / 2.0)]
        p4 = [self.position.x + m.cos(yaw) * (self.scale.x / 2.0) - m.sin(yaw) * (self.scale.y / 2.0),
              self.position.y + m.sin(yaw) * (self.scale.x / 2.0) + m.cos(yaw) * (self.scale.y / 2.0)]
        p5 = [self.position.x - m.cos(yaw) * (self.scale.x / 2.0) + m.sin(yaw) * (self.scale.y / 2.0),
              self.position.y - m.sin(yaw) * (self.scale.x / 2.0) - m.cos(yaw) * (self.scale.y / 2.0)]

        # p5 = np.array([999, 999])

        return np.array([
            p1, p2, p3, p4, p5
        ])
