#include "basecontrol/JoyTeleop.h"
using namespace JOYTELEOP;

JoyTeleop::JoyTeleop(std::string topic_name,bool publish_vel,double max_linear_velocity,double max_angular_velocity)
{
    control_trigger_=Default;
    publish_vel_=publish_vel;
    Joy_sub_  = nh_.subscribe(topic_name,100,&JoyTeleop::JoyCallback,this);
    if(publish_vel)
    {
        max_angular_velocity_=max_angular_velocity;
        max_linear_velocity_ = max_linear_velocity;
        joy_vel_pub = nh_.advertise<geometry_msgs::Twist>("/joy_vel", 1);
    }
    watchdog_timer_ = nh_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &JoyTeleop::watchdog, this, true);
    watchdog_timer_.start();
}

JoyTeleop::~JoyTeleop()
{

}

void JoyTeleop::watchdog(const ros::TimerEvent &e)
{
    ROS_WARN("joy not received for %f seconds, is the joy node drop?", WATCHDOG_PERIOD_);
    this->joy_alive_=false;
}

void JoyTeleop::JoyCallback(const sensor_msgs::JoyConstPtr &msg)
{   
    
    watchdog_timer_.stop();
    watchdog_timer_.start();
    if( msg -> buttons[0] ) //A
    {
        control_trigger_ = NaviPointSave;
    }
    else if( msg -> buttons[1] ) //B
    {
        control_trigger_ = Reset;
    }
    else if( msg -> buttons[2] ) //X
    {
        control_trigger_ = Init;
    }
    else if( msg -> buttons[3] ) //Y
    {
        control_trigger_ = Stop;
    }
    else if( msg -> axes[7] == -1 ) // down
    {
        control_trigger_ = Squat;
    }
    else if( msg -> axes[7] == 1 ) //up
    {
        control_trigger_ = Stand;
    }
    else if( msg -> axes[6] ==1 ) //left
    {
        control_trigger_ = Kneecut;
    }
    else if( msg -> axes[6] == -1) //right
    {
        control_trigger_ = Anklecut;
    }
    else if(msg->buttons[4]) //L1
    {
        control_trigger_ = Imu;
    }
    //else
    //{
    //    control_trigger_ = Default;
    //}
    
    if(publish_vel_)
    {
        geometry_msgs::Twist vel;

        if(msg->axes[1] != 0 || msg->axes[0] != 0)// yaogan_zuo 
        {
            vel.angular.z = msg->axes[0] * max_angular_velocity_;
            vel.linear.x = msg->axes[1] * max_linear_velocity_;
        }
        else if(msg->axes[3] != 0 || msg->axes[4] != 0)// yaogan_you
        {
            //vel.angular.z = msg->axes[3] * max_angular_velocity_;
        }
        joy_vel_pub.publish(vel);
    }
    
}

