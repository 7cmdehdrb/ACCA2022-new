#ifndef __MAP_TO_LOCAL_TF__
#define __MAP_TO_LOCAL_TF__

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_msgs/NDTStat.h>

namespace map_to_local_tf
{
    class map2localTF
    {
        public:
            map2localTF();
            ~map2localTF();
            void callbackNdtPose(const geometry_msgs::PoseStampedConstPtr& ptr);
            // void callbackNdtReliability(const std_msgs::Float32 rel);
            void callbackNdtStat(const autoware_msgs::NDTStat stat);

        private:
            ros::NodeHandle nh;
            //Publisher
            ros::Publisher _pub_ndt_pose_local;
            ros::Publisher _check_tf;
            //Subscriber
            ros::Subscriber _sub_ndt_pose;
            // ros::Subscriber _sub_ndt_reliabilty;
            ros::Subscriber _sub_ndt_stat;

            bool has_converged_;
            bool set_local_tf_;
            int num_matched_;
            float x;
            float y;
            float z;
            float w;

            tf2::Transform transform;
            tf2::Quaternion current_q;
            tf2_ros::StaticTransformBroadcaster br;
            geometry_msgs::TransformStamped map_to_local;
            geometry_msgs::Pose ndt_pose;
            ros::Time current_scan_time;
            geometry_msgs::PoseStamped ndt_pose_local;
            std_msgs::Bool check_tf;
          

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            tf2_ros::Buffer tf_buffer_;
    };
}

#endif