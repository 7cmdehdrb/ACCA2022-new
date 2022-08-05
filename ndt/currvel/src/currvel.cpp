#include "currvel.hpp"

namespace currvel
{
  currvel::currvel()
  {
    _sub_fb = nh.subscribe("erp42_feedback", 1, &currvel::callbackFB, this);
    _pub_cur_vel = nh.advertise<geometry_msgs::TwistStamped>("/current_velocity",1);

  }
  currvel::~currvel()
  {}

  void currvel::callbackFB(const erp42_msgs::SerialFeedBack& data)
  {
    // std::cout << "GET Ndt pose!" << std::endl;
    geometry_msgs::TwistStamped cv;
    cv.twist.linear.x = data.speed;
    _pub_cur_vel.publish(cv);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "currvel");

  currvel::currvel cv;

  ros::spin();

  return 0;
}
