#!/usr/bin/env python

from tokenize import Pointfloat
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
        self.dx = vel * m.cos(yaw-1.57079632679)
        self.dy = vel * m.sin(yaw-1.57079632679)
        
class erp42_Odometry(object):
    def __init__(self):
        # Subsriber
        rospy.Subscriber("/erp42_feedback", SerialFeedBack, callback  = self.encoderCallback)
        rospy.Subscriber("/imu/data", Imu, callback=self.imuCallback)
        
        # msg
        self.SerialFeedBack = SerialFeedBack()
        self.imuMsg = Imu()
        
        # param
        self.doTransform = True
        
        
    def encoderCallback(self, msg):
        self.SerialFeedBack = msg
    
    def imuCallback(self, msg):
        self.imuMsg = msg

        if state.init_yaw == 0.0:
            x = self.imuMsg.orientation.x
            y = self.imuMsg.orientation.y
            z = self.imuMsg.orientation.z
            w = self.imuMsg.orientation.w
            _, _, Yaw = tf.transformations.euler_from_quaternion([x, y, z, w])
            state.init_yaw = Yaw

            t = self.imuMsg.header.seq
            
    def handleData(self):
        vel = self.SerialFeedBack.speed
        
        x = self.imuMsg.orientation.x
        y = self.imuMsg.orientation.y
        z = self.imuMsg.orientation.z
        w = self.imuMsg.orientation.w
        t = self.imuMsg.header.seq
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
    position_x = 0
    position_y = 0
    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        current_time = time.time()
        dt = current_time - last_time
        state.update()
        odom.twist.twist = Twist(Vector3(state.dx, state.dy, 0.0), Vector3(0.0, 0.0, 0.0))
        position_x += float(state.dx) * dt
        position_y += float(state.dy) * dt
        odom.pose.pose.position.x = position_x
        odom.pose.pose.position.y = position_y
        odom.header.stamp.secs = current_time
        last_time = current_time
        odom_pub.publish(odom)
        r.sleep()