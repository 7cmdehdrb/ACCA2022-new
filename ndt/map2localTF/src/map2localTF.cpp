#include "map2localTF.hpp"

// #define LOW_COV [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
// #define HIH_COV [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

namespace map_to_local_tf
{
  map2localTF::map2localTF(): has_converged_(false), set_local_tf_(false)
  {
    _sub_ndt_pose = nh.subscribe("ndt_pose", 1, &map2localTF::callbackNdtPose, this);
    // _sub_ndt_reliabilty = nh.subscribe("ndt_reliability", 1, &map2localTF::callbackNdtReliability, this);
    _sub_ndt_stat = nh.subscribe("ndt_stat", 1, &map2localTF::callbackNdtStat, this);
    // _sub_ndt_stat = nh.subscribe("ndt_stat", 10, &map2localTF::callbackNdtStat, this);
    _pub_ndt_pose_local = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose_local", 1);
    _check_tf = nh.advertise<std_msgs::Bool>("/check_tf",1);
  }
  map2localTF::~map2localTF()
  {

  }

  void map2localTF::callbackNdtPose(const geometry_msgs::PoseStampedConstPtr& ptr)
  {
    std::cout << "GET Ndt pose!" << std::endl;

    current_scan_time = ptr->header.stamp;
    // current_scan_time = ros::Time().now();

    ndt_pose = ptr->pose;
    
    // current_q = tf2::Quaternion(ptr->pose.orientation.x, ptr->pose.orientation.y, ptr->pose.orientation.z);
  }

void map2localTF::callbackNdtStat(const autoware_msgs::NDTStat stat)
  {
    if(stat.score <= 0.15 && stat.score >= 0){
std::cout << "[1-1] not over 10 or -10 !" << std::endl;
      num_matched_++;
    } 
    else{
      num_matched_ = 0;
      // x = 0;
      // y = 0;
      // z = 0;
      // w = 0;
std::cout << "[1-2] over 10 or -10 !" << std::endl;
    }

    if(num_matched_ >= 10 ){
      has_converged_ = true;
      std::cout << "[2-1] has converged !" << std::endl;
    }
    else{
      has_converged_ = false;
      // x += ndt_pose.orientation.x;
      // y += ndt_pose.orientation.y;
      // z += ndt_pose.orientation.z;
      // w += ndt_pose.orientation.w;
      std::cout << "[2-2] has not converged !" << std::endl;
    }

    if(has_converged_ == true && set_local_tf_ == false ){
      set_local_tf_ = true;
      std::cout << "[3] make tf !" << std::endl;
      // Send TF "base_link" to "map"
      transform.setOrigin(tf2::Vector3(ndt_pose.position.x, ndt_pose.position.y, ndt_pose.position.z));
      current_q = tf2::Quaternion(ndt_pose.orientation.x, ndt_pose.orientation.y, ndt_pose.orientation.z, ndt_pose.orientation.w);
    
      // current_q = tf2::Quaternion(x/num_matched_, y/num_matched_, z/num_matched_, w/num_matched_);
      transform.setRotation(current_q);
      
      tf2::Stamped<tf2::Transform> tf(transform, ros::Time().now(), "map");
      geometry_msgs::TransformStamped tf_msg = tf2::toMsg(tf);
      tf_msg.child_frame_id = "local";
      br.sendTransform(tf_msg);
      check_tf.data = true;
      _check_tf.publish(check_tf);
      
    }    
    if(has_converged_ == true && set_local_tf_ == true){  // when local tf is set
      std::cout << "[4] pub !" << std::endl;
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
      map_to_local = tf_buffer_.lookupTransform("local", "map", ros::Time().now(), ros::Duration(1.0));
      tf2::doTransform(ndt_pose, ndt_pose_local.pose, map_to_local);
      // ndt_pose_local.header.stamp =  ros::Time().now();
      // ndt_pose_local.header.frame_id = "local";
      _pub_ndt_pose_local.publish(ndt_pose_local);
    }
  }

//   void map2localTF::callbackNdtReliability(const std_msgs::Float32 rel)
//   {
//     if(rel.data <= 10 && rel.data >= -10){
// std::cout << "[1-1] not over 10 or -10 !" << std::endl;
//       num_matched_++;
//     } 
//     else{
//       num_matched_ = 0;
//       // x = 0;
//       // y = 0;
//       // z = 0;
//       // w = 0;
// std::cout << "[1-2] over 10 or -10 !" << std::endl;
//     }

//     if(num_matched_ >= 10 ){
//       has_converged_ = true;
//       std::cout << "[2-1] has converged !" << std::endl;
//     }
//     else{
//       has_converged_ = false;
//       // x += ndt_pose.orientation.x;
//       // y += ndt_pose.orientation.y;
//       // z += ndt_pose.orientation.z;
//       // w += ndt_pose.orientation.w;
//       std::cout << "[2-2] has not converged !" << std::endl;
//     }

//     if(has_converged_ == true && set_local_tf_ == false ){
//       set_local_tf_ = true;
//       std::cout << "[3] make tf !" << std::endl;
//       // Send TF "base_link" to "map"
//       transform.setOrigin(tf2::Vector3(ndt_pose.position.x, ndt_pose.position.y, ndt_pose.position.z));
//       current_q = tf2::Quaternion(ndt_pose.orientation.x, ndt_pose.orientation.y, ndt_pose.orientation.z, ndt_pose.orientation.w);
    
//       // current_q = tf2::Quaternion(x/num_matched_, y/num_matched_, z/num_matched_, w/num_matched_);
//       transform.setRotation(current_q);
      
//       tf2::Stamped<tf2::Transform> tf(transform, ros::Time().now(), "map");
//       geometry_msgs::TransformStamped tf_msg = tf2::toMsg(tf);
//       tf_msg.child_frame_id = "local";
//       br.sendTransform(tf_msg);
//       check_tf.data = true;
//       _check_tf.publish(check_tf);
      
//     }    
//     if(has_converged_ == true && set_local_tf_ == true){  // when local tf is set
//       std::cout << "[4] pub !" << std::endl;
//       tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
//       map_to_local = tf_buffer_.lookupTransform("local", "map", ros::Time().now(), ros::Duration(1.0));
//       tf2::doTransform(ndt_pose, ndt_pose_local.pose, map_to_local);
//       // ndt_pose_local.header.stamp =  ros::Time().now();
//       // ndt_pose_local.header.frame_id = "local";
//       _pub_ndt_pose_local.publish(ndt_pose_local);
//     }
//   }

//   void map2localTF::callbackNdtStat(const autoware_msgs::NDTStat stat)
//   {
//     // if(stat.score <= 0.10 && stat.score >= 0){
//     //   ndt_pose_local->covariance = LOW_COV;
//     // }
//     // else{
//     //   ndt_pose_local->covariance = HIH_COV;
//     // }

//     if(has_converged_ == true && set_local_tf_ == true){  // when local tf is set
// std::cout << "[4] pub !" << std::endl;
//       tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
//       map_to_local = tf_buffer_.lookupTransform("local", "map", current_scan_time, ros::Duration(1.0));
//       tf2::doTransform(ndt_pose, ndt_pose_local.pose, map_to_local);
//       _pub_ndt_pose_local.publish(ndt_pose_local);
//     }
//   }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "map2localTF");

  map_to_local_tf::map2localTF m2l;

  ros::spin();

  return 0;
}
