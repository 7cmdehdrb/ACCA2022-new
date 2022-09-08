#include <stdio.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>


// Global Variables Define
tf::StampedTransform tf_map_to_odom;
geometry_msgs::Pose odom_pose;
geometry_msgs::Pose map_pose;

// Callback Functions
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_pose = msg->pose.pose;
}

void map_callback(const nav_msgs::Odometry::ConstPtr& msg){
    map_pose = msg->pose.pose;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_odom_tf_publisher");

    ros::NodeHandle nh;

    tf::TransformBroadcaster br;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    tf2::Transform latest_tf_;

    // Subscribers
    ros::Subscriber odom_sub = nh.subscribe("/p1", 10, odom_callback);
    ros::Subscriber map_sub = nh.subscribe("/p2", 10, map_callback);

    tf_map_to_odom.frame_id_ = "map";
    tf_map_to_odom.child_frame_id_ = "base_link";


    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {   
        tf_map_to_odom.stamp_ = ros::Time::now();

        geometry_msgs::PoseStamped odom_to_map;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0.7);
        tf2::Transform tmp_tf(q, tf2::Vector3(map_pose.position.x,
                                              map_pose.position.y,
                                              0.0));

        geometry_msgs::PoseStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = "base_link";
        tmp_tf_stamped.header.stamp = ros::Time::now();
        tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

        tf_->transform(tmp_tf_stamped, odom_to_map, "odom");

        tf2::convert(odom_to_map.pose, latest_tf_);

        // geometry_msgs::TransformStamped tmp_tf_stamped;
        // tmp_tf_stamped.header.frame_id = "map";
        // tmp_tf_stamped.header.stamp = ros::Time::now();
        // tmp_tf_stamped.child_frame_id = "odom";
        // tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);

        // br.sendTransform(tmp_tf_stamped);

        // tf_map_to_odom.setOrigin(
        //     tf::Vector3(
        //         map_pose.position.x - odom_pose.position.x, 
        //         map_pose.position.y - odom_pose.position.y, 
        //         0.0
        //     )
        // );

        // tf::Quaternion q;
        // q.setRPY(0, 0, 0.7);
        // // map r p y

        // tf::Quaternion qq;
        // tf::quaternionMsgToTF(odom_pose.orientation, qq);
        // q = q * qq.inverse();
        // tf_map_to_odom.setRotation(q);

        // br.sendTransform(tf_map_to_odom);

        // ros::spinOnce();

        ROS_INFO("HELLO WORLD!");
        r.sleep();
    }

    return 0;
}