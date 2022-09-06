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

<<<<<<< HEAD
    w = 0.8
    h = 2.0
=======
    w = 1.7
    h = 4.5
>>>>>>> 9d3c5abd012f89574b54c26b7bf764fa390c23fe

    parking_areas.append(ParkingArea(x=x, y=y, quat=quat, w=w, h=h))


def saveCallback(msg):
    global parking_areas

    path = rospkg.RosPack().get_path("parking") + "/parking/" + \
<<<<<<< HEAD
        rospy.get_param("/create_parking_area/parking_file", "parking.csv")
=======
        rospy.get_param("/create_parking_area/parking_file",
<<<<<<< HEAD
                        "hor_parking5.csv")
>>>>>>> 9d3c5abd012f89574b54c26b7bf764fa390c23fe
=======
                        "parking3.csv")
>>>>>>> 4f4b00fec7804e0cbe62f89c9b8558367516c1c6

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

    hz = 10
    freq = 1 / hz

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        msg = MarkerArray()

        for i, parking in enumerate(parking_areas):
            print(parking_areas)
            msg.markers.append(parking.parseMarker(id=i, duration=int(freq)))

        pub.publish(msg)
        r.sleep()
