#ifndef ACTIONLIB_H
#define ACTIONLIB_H
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/UInt32.h"
#include "fstream"
#include "chrono"
#include <iostream>



class ActionLib {
    public:
        enum ActionFlag{
            DropPrevention,
            Dodge,
            BackDown,
            FastStop,
            TakeTurn,
            Jump,
            Squat,
            Default 
        };
    private:
        ros::NodeHandle nh_;
        ros::Subscriber DropPrevention_Sub;
        ros::Publisher shuiping_depth;

        std_msgs::UInt32 last_depth;
        std_msgs::UInt32 latest_depth;
        long double dp1,dp_change;
        long double depth_value;
        

    public:
        explicit ActionLib(std::string serial_addr/*depth_msg*/);
        ~ActionLib();
        ActionFlag action_flag_;
        ActionFlag getActionFlag() {ActionFlag temp{action_flag_}; action_flag_ = Default; return temp;};   
        void DropPreventionCallback (const sensor_msgs::Image & image);
        void DropPrevent ();
};
#endif //ACTIONLIB_H