#!/usr/bin/env python

from mimetypes import init
from os import stat
from re import T
from tokenize import Pointfloat
from turtle import update

from keyring import delete_password
import rospy
import tf
import math as m
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose, Vector3, Point, Quaternion
from erp42_msgs.msg import SerialFeedBack
import time


class State():
    def __init__ (self, odom = None):        
        self.odometry = odom
        self.quat = [0.0, 0.0, 0.0, 1.0]
        self.v_quat = [0.0, 0.0, 0.0, 1.0]
        self.init_yaw = 0.0
    
    def update(self):

        vel, quat, t, v_quat = self.odometry.handleData()
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        _, _, v_yaw = tf.transformations.euler_from_quaternion(v_quat)
        # speed
        self.v = vel
        self.dx = vel * m.cos(yaw)
        self.dy = vel * m.sin(yaw)
        self.yaw = yaw
        self.v_yaw = v_yaw
        
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
            self.yaw2 = state.init_yaw
 

    def handleData(self):
        vel = self.SerialFeedBack.speed
        
        x = self.imuMsg.orientation.x
        y = self.imuMsg.orientation.y
        z = self.imuMsg.orientation.z
        w = self.imuMsg.orientation.w

        v_x = self.imuMsg.angular_velocity.x
        v_y = self.imuMsg.angular_velocity.y
        v_z = self.imuMsg.angular_velocity.z

        t = self.imuMsg.header.seq
        quat = [x, y, z, w]
        v_quat = [v_x, v_y, v_z, 1]

        return vel, quat, t, v_quat
    

if __name__== "__main__":

    odometry = erp42_Odometry()
    state = State(odom = odometry)

    rospy.init_node('erp42_odometry')
    odom_pub = rospy.Publisher("/odometry/wheel", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.1
    vy = 0
    th = 0.000001
    vth = 0.1

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(30.0)

    while not rospy.is_shutdown():

        state.update()

        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        print(dt)

        v = state.v / 3.6
        vth = state.v_yaw
        delta_th = vth * dt
        th += delta_th 

        # compute odometry in a typical way given the velocities of the robot


        delta_x = (v * m.cos(th)) * dt #
        delta_y = (v * m.sin(th)) * dt #

        vx = (v * m.cos(th)) #
        vy = (v * m.sin(th)) #

        x += delta_x 
        y += delta_y


        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "encoder",
            "base_link"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "encoder"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = ""
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
        odom.pose.covariance = [5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


        # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()