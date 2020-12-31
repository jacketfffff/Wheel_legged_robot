#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "ros/package.h"
#include "basecontrol/basecontrol.h"
#include "basecontrol/NaviCore.h"
#include "geometry_msgs/Pose2D.h"
#include "basecontrol/ActionLib.h"

struct targetPose{
    geometry_msgs::Pose2D pose;
    enum TargetAction{
        TAP,
        CHARGE,
        NAV,
        TURN 
    }targetAction{};
};
        
void getPoseArray(std::vector<targetPose> & targetPoseArray)
{
    const std::string pose_addr{ros::package::getPath("basecontrol")+"/config/Target.txt"};
    std::ifstream input_file(pose_addr.c_str());
    targetPoseArray.clear();
    if(input_file.is_open())
    {
        std::string str;
        targetPose target_pose{};
        while (getline(input_file,str) && !str.empty())
        {
            std::istringstream stringGet(str);
            int temp;
            stringGet >> temp >> target_pose.pose.x >> target_pose.pose.y >> target_pose.pose.theta;
            target_pose.targetAction = targetPose::TargetAction(temp);
            targetPoseArray.push_back(target_pose);
        }
        input_file.close();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "simpleNav");
    ros::NodeHandle nh_("~");
    ros::Rate loop_rate(60);

    //bool drop_prevention{false},control_mode{false}, nav_mode{false},force_change{false};

    const std::string parameter_addr{ros::package::getPath("basecontrol")+"/config/basemodel.yaml"};
    std::string base_foot_print, odom_frame, map_frame, serial_addr;
    bool publish_tf;

    nh_.param("base_foot_print", base_foot_print,(std::string)"base_link");
    nh_.param("odom_frame", odom_frame, (std::string)"odom");
    nh_.param("map_frame", map_frame, (std::string)"map");
    nh_.param("publish_tf", publish_tf, (bool)false);
    nh_.param("serial_addr", serial_addr, (std::string)"/dev/ttyUSB0");

    
    //basecontrol
  
    BaseController baseController(serial_addr,B115200,base_foot_print,odom_frame,publish_tf);
    baseController.setBaseModel(parameter_addr);
    //joy
    double max_linear_velocity, max_angular_velocity;
    nh_.param("max_linear_velocity", max_linear_velocity,(double)0.5);
    nh_.param("max_angular_velocity", max_angular_velocity, (double)0.4);
    JOYTELEOP::JoyTeleop joyTeleop("joy",true,max_linear_velocity,max_angular_velocity);
    //navigation
      std::cout << "11111!!!!!!!!!!!!!!!" << std::endl;
    NavCore navCore(base_foot_print,map_frame);
    //actionlib
    std::cout << "33333!!!!!!!!!!!!!!!" << std::endl;

    bool nav_on{true},nav_pause{},newGoal{true};
    std::vector<targetPose> targetPoseArray{};
    getPoseArray(targetPoseArray);
    auto iter = targetPoseArray.begin();
    std::cout << "2222!!!!!!!!!!!!!!!" << std::endl;
    
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while(ros::ok())
    {
        switch(joyTeleop.getControlTrigger())
        {
            case JOYTELEOP::Reset:
                //TODO reset
                std::cout << "Reset" << std::endl;
                break;//A
            case JOYTELEOP::Jump:
                //TODO jump
                std::cout << "Jump" << std::endl;
                break;//B
           
            case JOYTELEOP::Squat: 
                std::cout << "Squat" << std::endl;
                baseController.SquatController(100);
                break;//Y
            case JOYTELEOP::MoveForward: 
                baseController.sendCommand(baseController.MOVEFORWARD);
                break;//up
            case JOYTELEOP::MoveBack:
                baseController.sendCommand(baseController.MOVEBACK);
                break;//down
            case JOYTELEOP::TurnLeft:
                baseController.sendCommand(baseController.TURNLEFT);
                break;//left
            case JOYTELEOP::TurnRight:
                baseController.sendCommand(baseController.TURNRIGHT);
                break;//right
        }
        switch (navCore.getMoveBaseActionResult())
        {
            case NavCore::SUCCEEDED:
            {
                iter++;
                newGoal = true;
                break;
            }
            case NavCore::ABORTED:
            {
                ROS_ERROR_STREAM("ROBOT ABORTED ");
                
                
                    iter++;
                    newGoal=true;
               
                break;
            }
            default:
                break;
        }
       /* switch (actionLib.getActionFlag())
        {
            case ActionLib::DropPrevention: 
                baseController.sendCommand(baseController.STOP);
                break;
            case ActionLib::Dodge:
                //TODO  dodge
                break;
            case ActionLib::BackDown:
                //TODO  backdown
                break;
            case ActionLib::FastStop:
                //TODO FASTSTOP
                break;
            case ActionLib::TakeTurn:
                //TODO
                break;
            case ActionLib::Jump:
                //TODO  
                break;
            case ActionLib::Squat:
                //TODO  
                break;
        }*/
        if (nav_on && !targetPoseArray.empty() && newGoal)
        {
            if (iter != targetPoseArray.end())
            {
                ROS_INFO_STREAM("now goal : x is " << (*iter).pose.x << "y is " << (*iter).pose.y << "theta is " << (*iter).pose.theta);
                navCore.setGoal((*iter).pose);
            }
            else
            {
                getPoseArray(targetPoseArray);
                iter = targetPoseArray.begin();
                ROS_INFO("reach the end of the goal list, reload target");
                nav_on = false;
            }
            newGoal = false;
            
        }
       loop_rate.sleep();
       ros::spinOnce(); 
    }

}
