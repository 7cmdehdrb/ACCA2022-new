#!/usr/bin/env python


import rospy
import rospkg
import numpy as np
import math as m
# import tf
import csv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import *
from std_msgs.msg import Empty, Int8
from parking_area import ParkingArea
from time import sleep


def loadCSV(path):

    res = []

    with open(path, "r") as csvFile:
        reader = csv.reader(csvFile, delimiter=",")
        for row in reader:
            row = map(float, row)
            res.append(ParkingArea(x=row[0], y=row[1], quat=Quaternion(
                row[2], row[3], row[4], row[5]), h=row[6], w=row[7]))

    return res


def sequence_callback(msg):
    global sequence
    sequence = msg


if __name__ == "__main__":
    rospy.init_node("load_parking_area")

    rospy.wait_for_message('/parking_sequence', Int8)
    parking_sequence_sub = rospy.Subscriber(
        '/parking_sequence', Int8, callback=sequence_callback)
    sleep(0.1)
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
>>>>>>> e0fce322f9d4e8e5d7d34341f6284d8e0cc3f911

    pub = rospy.Publisher("/parking_areas", MarkerArray, queue_size=1)

    parking_areas = loadCSV(path)

    hz = 1.
    freq = 1 / hz

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        msg = MarkerArray()

        for i, parking in enumerate(parking_areas):
            msg.markers.append(parking.parseMarker(id=i, duration=int(freq)))

        pub.publish(msg)

        r.sleep()
