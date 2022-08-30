#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf


# y = np.arange(-3,3,0.5)
# x1 = y-1
# x = (y - 1)/0.5
# print(len(y))
# print(y)
# print(x1)
# print(len(x))

# test = np.polyfit(x, y, 1)
# print("test",test)

# for i in range(len(y)-1):
#     print(i)
#     test = np.polyfit([x[i],x[i+1]], [y[i], y[i+1]], 1)
#     print(test)
#     trajectory = test[0]*x + test[1]
#     print(trajectory)
#     print(len(trajectory))
#     print(type(trajectory))
#     print("-------------------------------------------------------")
    

import rospy
import time
import csv
from time import sleep
from DB import *
from path_plan.msg import PathRequest, PathResponse
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped, Point





db_name = rospy.get_param("/LoadPath/db_name", "/path.db")

if __name__ == "__main__":
    rospy.init_node("Test")
    # db = DB(db_name)
    # path = []
    # path_info1 = db.bring_pathinfo('B1C1')
    # path_info2 = db.bring_pathinfo('C1D1')
    # for info in path_info1:
    #     path.append(info)
    # for info in path_info2:
    #     path.append(info)

    # print(len(path_info2))

    # print(type(path))
    # print(len(path))
    # tf_sub = tf.TransformListener()


    # point_stamped = PointStamped()

    # point = Point()
    # point.x = 5
    # point.y = 7
    # point.z = 0

    # point_stamped.point = point
    
    # if tf_sub.canTransform("base_link", "map", rospy.Time(0)):
    #     goal_point = tf_sub.transformPoint(ps=point_stamped, target_frame="base_link")
    #     print(goal_point)
    # else:
    #     rospy.logwarn("Cannnot lookup transform between map and base_link : test.py")

    a = Point(1,2,3)
    print(a.x)
    print(a)    

