#!/usr/bin/env python

from mimetypes import init
from os import stat
from tokenize import Pointfloat
from turtle import update

from keyring import delete_password
import rospy
import tf
import math as m
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose, Vector3
from erp42_msgs.msg import SerialFeedBack
import time


class State():
    def __init__ (self, odom = None):        
        self.odometry = odom
        self.quat = [0.0, 0.0, 0.0, 1.0]
        
        self.init_yaw = 0.0
    
    def update(self):

        vel, quat, t = self.odometry.handleData()
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        
        # speed
        self.v = vel
        self.dx = vel * m.cos(yaw)
        self.dy = vel * m.sin(yaw)

        self.current_yaw = yaw
        
class erp42_Odometry(object):
    def __init__(self):
        # Subsriber
        rospy.Subscriber("/erp42_feedback", SerialFeedBack, callback  = self.encoderCallback)
        rospy.Subscriber("/odometry/global", Odometry, callback=self.global_odomCallback)
        # msg
        self.SerialFeedBack = SerialFeedBack()
        self.globalOdom = Odometry()
        
        # param
        self.doTransform = True
        
        
    def encoderCallback(self, msg):
        self.SerialFeedBack = msg

    def global_odomCallback(self, msg):
        self.globalOdom = msg

        if state.init_yaw == 0.0:
            x = self.globalOdom.pose.pose.orientation.x
            y = self.globalOdom.pose.pose.orientation.y
            z = self.globalOdom.pose.pose.orientation.z
            w = self.globalOdom.pose.pose.orientation.w
            _, _, Yaw = tf.transformations.euler_from_quaternion([x, y, z, w])
            state.init_yaw = Yaw
            t = self.globalOdom.header.seq
 
    def handleData(self):
        vel = self.SerialFeedBack.speed
        
        x = self.globalOdom.pose.pose.orientation.x
        y = self.globalOdom.pose.pose.orientation.y
        z = self.globalOdom.pose.pose.orientation.z
        w = self.globalOdom.pose.pose.orientation.w
        t = self.globalOdom.header.seq
        
        quat = [x, y, z, w]
        return vel, quat, t
    

if __name__== "__main__":
    rospy.init_node('erp42_odometry')
    odometry = erp42_Odometry()

    state = State(odom = odometry)
    odom_pub = rospy.Publisher( "/odometry/wheel", Odometry, queue_size=2)
    odom = Odometry()
    odom.header.frame_id = "odom"
    #odom.child_frame_id = "base_link"
    last_time = time.time()
    r = rospy.Rate(30.0)
    position_x = 0
    position_y = 0
    
    while not rospy.is_shutdown():

        current_time = time.time()
        dt = current_time - last_time

        state.update()

        position_x += float(state.dx * dt)
        position_y += float(state.dy * dt)

        odom.twist.twist = Twist(Vector3(state.dx, state.dy, 0.0), Vector3(0.0, 0.0, 0.0))
        odom.pose.pose.position.x = position_x
        odom.pose.pose.position.y = position_y
        odom.pose.pose.orientation.z = state.current_yaw
        odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        odom.header.stamp = rospy.Time.now()
        last_time = current_time
        odom_pub.publish(odom)

        r.sleep()