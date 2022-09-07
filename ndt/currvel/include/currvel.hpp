#ifndef __CURRVEL__
#define __CURRVEL__

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <erp42_msgs/SerialFeedBack.h>

namespace currvel
{
    class currvel
    {
        public:
            currvel();
            ~currvel();
            void callbackFB(const erp42_msgs::SerialFeedBack& ptr);

        private:
            ros::NodeHandle nh;
            //Publisher
            ros::Publisher _pub_cur_vel;
            //Subscriber
            ros::Subscriber _sub_fb;
    };
}

#endif