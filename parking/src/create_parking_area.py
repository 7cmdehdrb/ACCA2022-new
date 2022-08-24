#!/usr/bin/env python


import rospy
import rospkg
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import *
from std_msgs.msg import Empty
from parking_area import ParkingArea


def poseCallback(msg):
    global parking_areas

    x = msg.pose.position.x
    y = msg.pose.position.y

    quat = msg.pose.orientation

    w = 1.7
    h = 4.5

    parking_areas.append(ParkingArea(x=x, y=y, quat=quat, w=w, h=h))


def saveCallback(msg):
    global parking_areas

    path = rospkg.RosPack().get_path("parking") + "/parking/" + \
        rospy.get_param("/create_parking_area/parking_file",
                        "parking2.csv")

    with open(path, 'w') as csvfile:
        for parking in parking_areas:

            position = parking.position
            orientation = parking.orientation
            scale = parking.scale

            # x, y, ox, oy, oz, ow, h, w
            csvfile.write(
                str(position.x) + "," +
                str(position.y) + "," +
                str(orientation.x) + "," +
                str(orientation.y) + "," +
                str(orientation.z) + "," +
                str(orientation.w) + "," +
                str(scale.x) + "," +
                str(scale.y) + "\n"
            )

        rospy.loginfo("Save Parking Area! : %s" % path)


if __name__ == "__main__":
    rospy.init_node("create_parking_area")

    parking_areas = []

    sub = rospy.Subscriber("/move_base_simple/goal",
                           PoseStamped, callback=poseCallback)
    save = rospy.Subscriber("/save_parking", Empty, callback=saveCallback)
    pub = rospy.Publisher("/parking_areas", MarkerArray, queue_size=1)

    hz = 1
    freq = 1 / hz

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        msg = MarkerArray()

        for i, parking in enumerate(parking_areas):
            msg.markers.append(parking.parseMarker(id=i, duration=int(freq)))

        pub.publish(msg)

        r.sleep()
