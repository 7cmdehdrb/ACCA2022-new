#include <stdio.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>


// Global Variables Define
static tf2_ros::TransformBroadcaster br;
static tf::StampedTransform tf_map_to_odom;
static geometry_msgs::Pose odom_pose;
static geometry_msgs::Pose map_pose;

// Callback Functions
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_pose = msg->pose.pose;
}

void map_callback(const nav_msgs::Odometry::ConstPtr& msg){
    map_pose = msg->pose.pose;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_odom_tf_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Subscribers
    ros::Subscriber odom_sub = nh.subscribe("/odometry/kalman", 10, odom_callback);
    ros::Subscriber map_sub = nh.subscribe("/ndt_matching/ndt_pose", 10, map_callback);

    tf_map_to_odom.frame_id_ = "map";
    tf_map_to_odom.child_frame_id_ = "odom";

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {   

        tf_map_to_odom.stamp_ = ros::Time::now();

        tf_map_to_odom.setOrigin(
            tf::Vector3(
                map_pose.position.x - odom_pose.position.x, 
                map_pose.position.y - odom_pose.position.y, 
                0.0
            )
        );

        tf::Quaternion q;
        q.setRPY(0, 0, 0);

        tf::Quaternion qq;
        tf::quaternionMsgToTF(odom_pose.orientation, qq);
        q = q * qq.inverse();
        tf_map_to_odom.setRotation(q);

        br.sendTransform();

        // br.sendTransform(tf_map_to_odom);

        ROS_INFO("HELLO WORLD!");
        r.sleep();
    }

    return 0;
}