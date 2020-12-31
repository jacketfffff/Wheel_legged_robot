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
    if( msg -> buttons[0] == 1 && msg -> buttons[4] == 0  && msg -> buttons[5] == 0) //A
    {
        control_trigger_ = NaviPointSave;
    }
    else if( msg -> buttons[1] && msg -> buttons[4] == 0 && msg -> buttons[5] == 0) //B
    {
        control_trigger_ = Reset;
    }
    else if( msg -> buttons[2] && msg -> buttons[4] == 0 && msg -> buttons[5] == 0) //X
    {
        control_trigger_ = Init;
    }
    else if( msg -> buttons[3] && msg -> buttons[4] == 0 && msg -> buttons[5] == 0) //Y
    {
        control_trigger_ = ClearReset;
    }
    else if( msg -> axes[7] == -1 && msg -> buttons[4] == 0 && msg -> buttons[5] == 0) // down
    {
        control_trigger_ = Squat;
    }
    else if( msg -> axes[7] == 1 && msg -> buttons[4] == 0 && msg -> buttons[5] == 0) //up
    {
        control_trigger_ = Stand;
    }
    else if( msg -> axes[6] ==1 && msg -> buttons[4] == 0 && msg -> buttons[5] == 0) //left
    {
        control_trigger_ = AnkleLock;
    }
    else if( msg -> axes[6] == -1 && msg -> buttons[4] == 0 && msg -> buttons[5] == 0) //right
    {
        control_trigger_ = Anklecut;
    }
    /* L1 + other triggers */
    else if( msg -> buttons[0] == 1 && msg -> buttons[4] == 1  && msg -> buttons[5] == 0) //A + L1
    {
        
    }
    else if( msg -> buttons[1] && msg -> buttons[4] == 1 && msg -> buttons[5] == 0) //B + L1
    {
        
    }
    else if( msg -> buttons[2] && msg -> buttons[4] == 1 && msg -> buttons[5] == 0) //X + L1
    {
        
    }
    else if( msg -> buttons[3] && msg -> buttons[4] == 1 && msg -> buttons[5] == 0) //Y + L1
    {
        
    }
    else if( msg -> axes[7] == -1 && msg -> buttons[4] == 1 && msg -> buttons[5] == 0) // down + L1
    {
        
    }
    else if( msg -> axes[7] == 1 && msg -> buttons[4] == 1 && msg -> buttons[5] == 0) //up + L1
    {
        
    }
    else if( msg -> axes[6] ==1 && msg -> buttons[4] == 1 && msg -> buttons[5] == 0) //left + L1
    {
        control_trigger_ =  KneeLock;
    }
    else if( msg -> axes[6] == -1 && msg -> buttons[4] == 1 && msg -> buttons[5] == 0) //right + L1
    {
        control_trigger_ = Kneecut;
    }
    /* R1 + other triggers */
     else if( msg -> buttons[0] == 1 && msg -> buttons[4] == 0  && msg -> buttons[5] == 1) //A + L2
    {
        
    }
    else if( msg -> buttons[1] && msg -> buttons[4] == 0 && msg -> buttons[5] == 1)//B + L2
    {
        
    }
    else if( msg -> buttons[2] && msg -> buttons[4] == 0 && msg -> buttons[5] == 1) //X + L2
    {
        
    }
    else if( msg -> buttons[3] && msg -> buttons[4] == 0 && msg -> buttons[5] == 1) //Y + L2
    {
        
    }
    else if( msg -> axes[7] == -1 && msg -> buttons[4] == 0 && msg -> buttons[5] == 1) // down + L2
    {
        
    }
    else if( msg -> axes[7] == 1 && msg -> buttons[4] == 0 && msg -> buttons[5] == 1) //up + L2
    {
        
    }
    else if( msg -> axes[6] ==1 && msg -> buttons[4] == 0 && msg -> buttons[5] == 1) //left + L2
    {
        control_trigger_ = HipLock;
    }
    else if( msg -> axes[6] == -1 && msg -> buttons[4] == 0 && msg -> buttons[5] == 1) //right + L2
    {
        control_trigger_ = Hipcut;
    }
    /* start and select */
    else if( msg -> buttons[7] == 1)//start
    {
        control_trigger_ = Imustart;
    }
    else if( msg -> buttons[6] == 1)//select
    {
        control_trigger_ == Stop;
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

