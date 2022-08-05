#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

// static int REL_ARR_LEN = 10;
static bool _has_converged;
static bool _set_local_tf;
static int _num_matched;
static tf2::Transform transform;
static ros::Time current_scan_time;
static tf2::Quaternion current_q;
static geometry_msgs::Pose current_pose;
static tf2_ros::StaticTransformBroadcaster br;
// static std::vector<double> ndt_reliability_arr;
// static double ndt_reliability_arr[REL_ARR_LEN];

static void ndt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& input){
  current_scan_time = input->header.stamp;
  current_pose = input->pose;
  current_q = tf2::Quaternion(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z);
  // current_q.x = input->pose.orientation.x;
  // current_q.y = input->pose.orientation.y;
  // current_q.z = input->pose.orientation.z;
}

static void ndt_rel_callback(const std_msgs::Float32 rel)
{
  if(rel.data <= 10 && rel.data >= -10){
ROS_INFO("1");
    _num_matched ++;
  } 
  else{
    _num_matched = 0;
ROS_INFO("2");
  }

  if(_num_matched >= 10){
    _has_converged = true;
ROS_INFO("3");
  }
  else{
    _has_converged = false;
    _set_local_tf = false;
ROS_INFO("4");
  }

  if (_has_converged == true && _set_local_tf == false){
    _set_local_tf = true;
ROS_INFO("here");
    // Send TF "base_link" to "map"
    transform.setOrigin(tf2::Vector3(current_pose.position.x, current_pose.position.y, current_pose.position.z));
    transform.setRotation(current_q);

    tf2::Stamped<tf2::Transform> tf(transform, current_scan_time, "map");
    geometry_msgs::TransformStamped tf_msg = tf2::toMsg(tf);
    tf_msg.child_frame_id = "local";
    br.sendTransform(tf_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map2localTF");
  if (!ros::isInitialized())
  {
    ROS_INFO("not initialized");
    return 0;
  }
  ros::NodeHandle nh;

  // init
  _num_matched = 0;
  _has_converged = false;
  _set_local_tf = false;

  ros::Subscriber ndt_pose_sub = nh.subscribe("ndt_pose", 10, ndt_pose_callback);
  ros::Subscriber ndt_rel_sub = nh.subscribe("ndt_reliablity", 10, ndt_rel_callback);
  
  ros::spin();
  
  return 0;
}   
    