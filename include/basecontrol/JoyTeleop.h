#ifndef JOYTELEOP_H
#define JOYTELEOP_H
#include <utility>

#include"ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

namespace JOYTELEOP
{
    enum ControlTrigger{
        Default,
        Reset,
        MoveForward,
        MoveBack,
        TurnLeft,
        TurnRight,
        Squat,
        Jump,
        JointLock,
        NaviPointSave,
        Init,
        Stop,
        Stand,
        Anklecut,
        Kneecut,
        Imu
    };
    class JoyTeleop{
        private:
        const std::string TOPIC_NAME_;
        const double WATCHDOG_PERIOD_=2.0;

        ros::NodeHandle nh_;
        ros::Timer watchdog_timer_;
        ros::Subscriber Joy_sub_;
        ros::Publisher joy_vel_pub;
        bool joy_alive_;
        bool publish_vel_;
        ControlTrigger control_trigger_;
        double max_linear_velocity_;
        double max_angular_velocity_; 

        public:
        explicit JoyTeleop(std::string topic_name,bool publish_vel=false,double max_linear_velocity=0.2,double max_angular_velocity=0.4);
        ~JoyTeleop();
        void JoyCallback(const sensor_msgs::JoyConstPtr & msg);
        void watchdog(const ros::TimerEvent &e);
        ControlTrigger getControlTrigger()  {ControlTrigger temp{control_trigger_};control_trigger_=Default;return temp;};
    };
}

#endif  //JOYTELEOP_H