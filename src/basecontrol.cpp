//#include "ros/ros.h"
//#include "ros/package.h"
#include <unistd.h>
#include "basecontrol/basecontrol.h"
#include <sys/time.h>
#include "squat_planning.cpp"
using namespace JOYTELEOP;
using namespace cv;

BaseController::BaseController(std::string serial_addr, unsigned int baudrate, std::string base_foot_print, std::string odom_frame,std::string serial_addr1,std::string serial_addr2,bool publish_tf)
{   
    
    serialManager = new NaviSerialManager(serial_addr, baudrate,8);
    //if(serialManager -> openSerial())
    //{
        LeftForceSensor = new NaviSerialManager(serial_addr1,baudrate,12);
        LeftForceSensor -> openSerial();
        LeftForceSensor -> registerAutoReadThread(TIMER_SPAN_RATE_);

        RightForceSensor = new NaviSerialManager(serial_addr2,baudrate,12);
        RightForceSensor -> openSerial();
        RightForceSensor -> registerAutoReadThread(TIMER_SPAN_RATE_);

        left_forcesensor_pub = nh_.advertise<std_msgs::Float64MultiArray>("/left_forcesensor",100);
        right_forcesensor_pub = nh_.advertise<std_msgs::Float64MultiArray>("/right_forcesensor",100); 

        serialManager -> registerAutoReadThread(TIMER_SPAN_RATE_);

        wheel_status_pub = nh_.advertise<basecontrol::WheelStatus>("/wheel_status", 100);
        //odom_raw_pub = nh_.advertise<nav_msgs::Odometry>("/odom_raw",100);
        cmd_vel_sub = nh_.subscribe("/cmd_vel", 100, &BaseController::cmd_velCallback, this);
        joy_vel_sub = nh_.subscribe("/joy_vel", 100, &BaseController::joy_velCallback, this);
        simulator_cmd_pub = nh_.advertise <geometry_msgs::Twist>("/cmd_vel",100);
        DropPrevention_Sub = nh_.subscribe("/camera/depth/image_rect_raw",1000,&BaseController::DropPreventionCallback,this);
		multi_imu = nh_.subscribe("/robot_Pitch", 100, &BaseController::MultiImuCallback, this);
        //init_send_msgs();

        send_timer_ = nh_.createTimer(ros::Duration(1.0 / TIMER_SPAN_RATE_),&BaseController::sendtimerCallback,this);
        send_timer_.start();
        //read_timer_ = nh_.createTimer(ros::Duration(1.0/TIMER_SPAN_RATE_),&BaseController::readtimerCallback,this);
        //read_timer_.start();
        left_forcesensor_timer_ =  nh_.createTimer(ros::Duration(2/TIMER_SPAN_RATE_),&BaseController::leftforcesensorCallback,this);
        left_forcesensor_timer_.start();
        right_forcesensor_timer_ = nh_.createTimer(ros::Duration(2/TIMER_SPAN_RATE_),&BaseController::rightforcesensorCallback,this);
        right_forcesensor_timer_.start();
        //odom_publish_timer_ = nh_.createTimer(ros::Duration(1.0/ODOM_TIMER_SPAN_RATE_), &BaseController::odom_publish_timer_callback, this);
        //odom_publish_timer_.start();
        ROS_INFO_STREAM("BASE READY");
   // }
    //else
    //ROS_ERROR_STREAM("Can't open "<<"SERIAL "<<serial_addr<<std::endl);
}       

BaseController::~BaseController()
{

    delete serialManager;
    delete LeftForceSensor;
    delete RightForceSensor;
}
// leftforcesensorcallback  左腿力传感器数据处理    
void BaseController::leftforcesensorCallback( const ros::TimerEvent & e)
{
    NaviSerialManager::ReadResult self_results{LeftForceSensor->getReadResult()};
    if(self_results.read_bytes >= 12)
    {
        for (int i=0; i<self_results.read_bytes; i+= 12)
        {
            memcpy(leftforcesensor_, &self_results.read_result[i], 12);
             if (leftforcesensor_[0] == 0x49 && leftforcesensor_[10] == 0x0D && leftforcesensor_[11] == 0x0A)
             {
                 int Fx,Fy,Fz,Mx,My,Mz;
                if(leftforcesensor_[1]&0x80)
                {
                    Fx=(((~leftforcesensor_[1])&0x7F)<<4)+((~leftforcesensor_[2])>>4)+1;
                    Fx=-Fx;	
                }
                else
                 {
                    Fx=((leftforcesensor_[1]&0x7F)<<4)+(leftforcesensor_[2]>>4);
                 } 
                if(leftforcesensor_[2]&0x08)
                {
                    Fy=(((~leftforcesensor_[2])&0x07)<<8)+(~leftforcesensor_[3])+1;
                    Fy=-Fy;	
                }
                else
                {
                    Fy=((leftforcesensor_[2]&0x07)<<8)+leftforcesensor_[3];
                }	
                if(leftforcesensor_[4]&0x80)
                {
                    Fz=(((~leftforcesensor_[4])&0x7F)<<4)+((~leftforcesensor_[5])>>4)+1;
                    Fz=-Fz;	
                }
                else
                {
                    Fz=((leftforcesensor_[4]&0x7F)<<4)+(leftforcesensor_[5]>>4);
                } 
                if(leftforcesensor_[5]&0x08)
                {
                    Mx=(((~leftforcesensor_[5])&0x07)<<8)+(~leftforcesensor_[6])+1;
                    Mx=-Mx;	
                }
                else
                {
                    Mx=((leftforcesensor_[5]&0x07)<<8)+leftforcesensor_[6];
                }
                if(leftforcesensor_[7]&0x80)
                { 
                    My=(((~leftforcesensor_[7])&0x7F)<<4)+((~leftforcesensor_[8])>>4)+1;
                    My=-My;	
                }
                else
                {
                    My=((leftforcesensor_[7]&0x7F)<<4)+(leftforcesensor_[8]>>4);
                }
                if(leftforcesensor_[8]&0x08)
                {
                    Mz=(((~leftforcesensor_[8])&0x07)<<8)+(~leftforcesensor_[9]);
                    Mz=-Mz;	
                }
                else
                {
                    Mz=((leftforcesensor_[8]&0x07)<<8)+leftforcesensor_[9];
                }
                 Leftforcesensor_[0] = FORCE_TRANSFORM_ * Fx;
                 Leftforcesensor_[1] = FORCE_TRANSFORM_ * Fy; 
                 Leftforcesensor_[2] = FORCE_TRANSFORM_ * Fz;
                 Leftforcesensor_[3] = MOMENT_TRANSFORM_ * Mx;
                 Leftforcesensor_[4] = MOMENT_TRANSFORM_ * My;
                 Leftforcesensor_[5] = MOMENT_TRANSFORM_ * Mz;
                 std_msgs::Float64MultiArray Leftforcesensor;
                 Leftforcesensor.data.resize(6);
                 memcpy(Leftforcesensor.data.data(), &Leftforcesensor_[0], 6*sizeof(double));
                 left_forcesensor_pub.publish(Leftforcesensor);
             }
             else
             ROS_WARN_STREAM("LEFT_FORCESNESOR DATA ERROR");
        }
    }
}
// rightforcesensorcallback 右腿力传感器数据处理
void BaseController::rightforcesensorCallback( const ros::TimerEvent & e )
{   
    
    NaviSerialManager::ReadResult self_results{RightForceSensor->getReadResult()};
    if(self_results.read_bytes >= 12)
    {
        for (int i=0; i<self_results.read_bytes; i+= 12)
        {
            memcpy(rightforcesensor_, &self_results.read_result[i], 12);
             if (rightforcesensor_[0] == 0x49 && rightforcesensor_[10] == 0x0D && rightforcesensor_[11] == 0x0A)
             {
                 int Fx,Fy,Fz,Mx,My,Mz;
                if(rightforcesensor_[1]&0x80)
                {
                    Fx=(((~rightforcesensor_[1])&0x7F)<<4)+((~rightforcesensor_[2])>>4)+1;
                    Fx=-Fx;	
                }
                else
                 {
                    Fx=((rightforcesensor_[1]&0x7F)<<4)+(rightforcesensor_[2]>>4);
                 } 
                if(rightforcesensor_[2]&0x08)
                {
                    Fy=(((~rightforcesensor_[2])&0x07)<<8)+(~rightforcesensor_[3])+1;
                    Fy=-Fy;	
                }
                else
                {
                    Fy=((rightforcesensor_[2]&0x07)<<8)+rightforcesensor_[3];
                }	
                if(rightforcesensor_[4]&0x80)
                {
                    Fz=(((~rightforcesensor_[4])&0x7F)<<4)+((~rightforcesensor_[5])>>4)+1;
                    Fz=-Fz;	
                }
                else
                {
                    Fz=((rightforcesensor_[4]&0x7F)<<4)+(rightforcesensor_[5]>>4);
                } 
                if(rightforcesensor_[5]&0x08)
                {
                    Mx=(((~rightforcesensor_[5])&0x07)<<8)+(~rightforcesensor_[6])+1;
                    Mx=-Mx;	
                }
                else
                {
                    Mx=((rightforcesensor_[5]&0x07)<<8)+rightforcesensor_[6];
                }
                if(rightforcesensor_[7]&0x80)
                { 
                    My=(((~rightforcesensor_[7])&0x7F)<<4)+((~rightforcesensor_[8])>>4)+1;
                    My=-My;	
                }
                else
                {
                    My=((rightforcesensor_[7]&0x7F)<<4)+(rightforcesensor_[8]>>4);
                }
                if(rightforcesensor_[8]&0x08)
                {
                    Mz=(((~rightforcesensor_[8])&0x07)<<8)+(~rightforcesensor_[9]);
                    Mz=-Mz;	
                }
                else
                {
                    Mz=((rightforcesensor_[8]&0x07)<<8)+rightforcesensor_[9];
                }
                 Rightforcesensor_[0] = FORCE_TRANSFORM_ * Fx;
                 Rightforcesensor_[1] = FORCE_TRANSFORM_ * Fy; 
                 Rightforcesensor_[2] = FORCE_TRANSFORM_ * Fz;
                 Rightforcesensor_[3] = MOMENT_TRANSFORM_ * Mx;
                 Rightforcesensor_[4] = MOMENT_TRANSFORM_ * My;
                 Rightforcesensor_[5] = MOMENT_TRANSFORM_ * Mz;
                 std_msgs::Float64MultiArray Rightforcesensor;
                 Rightforcesensor.data.resize(6);
                memcpy(Rightforcesensor.data.data(), &Rightforcesensor_[0], 6*sizeof(double));
                right_forcesensor_pub.publish(Rightforcesensor);
             }
             else
             ROS_WARN_STREAM("RIGHT_FORCESNESOR DATA ERROR");
        }
    }
}
// droppreventioncallback 防跌落
void BaseController::DropPreventionCallback(const sensor_msgs::Image & image)
{
	drop_prevention_time_3 = ros::Time::now();    
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::TYPE_16UC1);
    Mat img = cv_ptr -> image;
    int m,n,k;
    int j = 0;
    unsigned int height  = img.rows;
    unsigned int width= img.cols;
    dp1 = depth_value;
    depth_value = 0;
    k=0;
    for(m=height/2 - 2 ; m <= height/2 + 2 ; m++ )
    {
        for (n = width/3; n<=width*2/3 ; n++)
            {
                depth_value += img.at<uint16_t>(m,n)*0.001f;
                k += 1;
            }
    }
    depth_value = depth_value/k;
    latest_depth.data=depth_value;
    //cout << "depth_value is " << depth_value << endl;
    
    dp_change = depth_value - dp1;
    dp_change = abs(dp_change);
	drop_prevention_time_4 = ros::Time::now();
 	//std::cout << "dp_change" << dp_change << std::endl;
    if(dp_change > 0.3)
        {   
            drop_prevention_time_1 = ros::Time::now();
            //std::cout << "drop_prevention_time_1 = " << drop_prevention_time_1 << std::endl;
            //serialManager -> send(wheel_vel, COMMAND_SIZE);
            drop_prevention_time_2 = ros::Time::now();
            //std::cout << "drop_prevention_time_2 = " << drop_prevention_time_2 << std::endl;
            std::cout << "Response Time = " << (drop_prevention_time_4 - drop_prevention_time_3).toSec()*10000 << "ms" << std::endl;
			std::cout<<"stop"<<std::endl;
        } 
}
//手柄速度处理
void BaseController::joy_velCallback(const geometry_msgs::TwistConstPtr &msg)
{
    if(cmd_vel_received_)
		cmd_vel_received_=false;
    //Cmd_vel user_cmd_vel;
    
    geometry_msgs::Twist simulator_cmd_;
    simulator_cmd_.linear.x = msg -> linear.x;
    simulator_cmd_.angular.z = msg -> angular.z;
    simulator_cmd_pub.publish(simulator_cmd_);
    
    
    double linear_velocity{msg->linear.x};
    double angular_velocity{msg->angular.z};
    double right_vel, left_vel;

    if(linear_velocity == 0)
    {
        right_vel = angular_velocity * BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel  = -right_vel;
    }
    else if(angular_velocity == 0)
        right_vel = left_vel = linear_velocity;
    else
    {
        right_vel  = linear_velocity + angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel   = linear_velocity - angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
    }

    joy_vel_received_ = linear_velocity!=0||angular_velocity!=0 ;
    //when there is no-zero cmd velocity, do not send zero joy velocity
    if(!cmd_vel_received_||joy_vel_received_)
    {
        velocity_mutex_.lock();
        user_cmd_vel_.cmd_right_wheel = right_vel*60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
        user_cmd_vel_.cmd_left_wheel  = left_vel *60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
        velocity_mutex_.unlock();
    }
    //cmd_vel_received_ = linear_velocity!=0||angular_velocity!=0 ;
    //ROS_INFO_STREAM("GET INFORMATION FROM JOY AND SEND VELOCITY");
}
// 导航速度处理
void BaseController::cmd_velCallback(const geometry_msgs::TwistConstPtr &msg)
{
    cmd_vel_watch_ = ros::Time::now();   
    if(joy_vel_received_)
        return;

    //Cmd_vel user_cmd_vel{};
    double linear_velocity{msg->linear.x};
    double angular_velocity{msg->angular.z};
    double right_vel{}, left_vel{};
    if(linear_velocity == 0)
    {
        right_vel = angular_velocity * BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel  = -right_vel;
    }
    else if(angular_velocity == 0)
        right_vel = left_vel = linear_velocity;
    else
    {
        right_vel  = linear_velocity + angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel   = linear_velocity - angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
    }

    cmd_vel_received_ = linear_velocity!=0||angular_velocity!=0 ;

    velocity_mutex_.lock();
    user_cmd_vel_.cmd_right_wheel = right_vel*60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
    user_cmd_vel_.cmd_left_wheel  = left_vel *60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
    //sendVelocity(user_cmd_vel_);
    velocity_mutex_.unlock();
}
//下蹲 暂时不用
void BaseController::SquatController(double rate)
{
    //double depth,double start_time,double step,double distance,double vel,double rate
    NaviMode = false;
    ControlMode = true;
    if (!ControlMode)
        return;
    std::cout << "thread" << endl;
    thread_ptr_.reset(new boost::thread(boost::bind(&BaseController::SquatThread, this,rate)));
    ThreadRegistered_=true;
    thread_ptr_->detach();
    
}
//下蹲线程 不用
void BaseController::SquatThread(double rate)
{
    try
    {
        struct timeval start;
        struct timeval end;
        squat_planning();
        int i = 0;
        //double joint_vel(10);
        bool send_flag = true;
        int n = sizeof(angle_a);
        while(send_flag)
        {

            gettimeofday(&start,NULL);
            send_joint_control(i);
            //TODO receive
            gettimeofday(&end,NULL);
            float time_use = (end.tv_sec-start.tv_sec)*1000000+(end.tv_usec-start.tv_usec);
            std::cout << time_use << std::endl;
            i = i + 1;
            if(i == 799)
            send_flag = false;
            usleep(10000 - (int)time_use);
            std::cout << "time_use" << (int)time_use << std::endl;
        }
    }
    catch(boost::thread_interrupted&e)
    {
        ControlMode = false;
        NaviMode = true;
        ROS_INFO("controlMode end !!");
        // TODO joint lock
    }
    
}
//下蹲规划 不用
void BaseController::squat_planning()
{
    std::cout << "squat_planning" << std::endl;
    using namespace std;
    double t_end,squat_t,delta_t,t_real,dY;
	double data_delta = 0.01;
	ExtU rtU;
	ExtY rtY;
    for (int i = 0;i <= 799;i++){  
	//double t_real;
	t_real = i * data_delta;
	rtU = {-100,2,t_real,3,1};
	t_end = rtU.X_d / rtU.forward_vel + rtU.t_sim0;
	squat_t = t_end - rtU.t_sim0;
	delta_t = rtU.t_sim-rtU.t_sim0;

	if (rtU.t_sim <= rtU.t_sim0) {
		dY = 0.0;
	}
	else if (rtU.t_sim <= t_end) {
		double a0,a1,a2,a3,a4,a5;
		a0 = 0;
		a1 = 0;
		a2 = 0;
		a3 = (20 * rtU.Y_d) * (1 / (2 * pow(squat_t,3)));
		a4 = (-30 * rtU.Y_d) * (1 / (2 * pow(squat_t,4)));
		a5 = (12 * rtU.Y_d) * (1 / (2 * pow(squat_t,5)));
		dY = a0+a1*pow(delta_t,1)+a2*pow(delta_t,2)+a3*pow(delta_t,3)+a4*pow(delta_t,4)+a5*pow(delta_t,5);
	}
	else {
		dY = rtU.Y_d;
	}
	//以上为笛卡尔轨迹规划

	double dr,rd,La,Lk,X0,Y0,k,qa0,qk0,qh0;
	dr = M_PI/180;
	rd = 180/M_PI;

	La = 188.4882;
	Lk = 222.0;
	X0 = 65.4;
	Y0 = 374.9;

	k = 0;
	qa0 = 104.0935;
	qk0 = 59.9112;
	qh0 = 105.3751;

	Eigen::Matrix<double, 2, 1> F;
	Eigen::Matrix<double, 2, 1> kcal_0;
	Eigen::Matrix<double, 2, 2> G;
	Eigen::Matrix<double, 2, 1> d_kcal;
	Eigen::Matrix<double, 3, 1> q_passive;

	kcal_0 << (qa0 - 0.213 * dY) * dr,\
			 (0.232 * dY + qk0) * dr;

	F << (La * cos(kcal_0(0,0)) + Lk * cos(kcal_0(1,0))) - X0,\
	     (La * sin(kcal_0(0,0)) + Lk * sin(kcal_0(1,0))) - (Y0 + dY);

	while(F.norm()>1E-10){
		F << (La * cos(kcal_0(0,0)) + Lk * cos(kcal_0(1,0))) - X0,\
		 (La * sin(kcal_0(0,0)) + Lk * sin(kcal_0(1,0))) - (Y0 + dY);
		G << -La * sin(kcal_0(0,0)),\
		 -Lk * sin(kcal_0(1,0)),\
		  La * cos(kcal_0(0,0)),\
		  Lk * cos(kcal_0(1,0));

		d_kcal = -G.inverse() * F;
		kcal_0 += d_kcal;
		k++;
	   if (k >= 100)
	   break;
	}
	q_passive << kcal_0(0,0) * rd,\
				 kcal_0(1,0) * rd,\
				 0;
	//以上为逆运动学解析

	double qm10,q_delta,qc10,qs10,qf10,qc1,delta_qa;
	qm10 = 204.1062;
	q_delta = 14.3573;
	qc10 = 89.7362;
	qs10 = 256.8924;
	qf10 = -6.6292;
	qc1 = q_passive(0,0)-q_delta;
    delta_qa = qc1-qc10;

	double Lm1,Ls1,Lf1,Lc1;
	Lm1 = 40;
	Ls1 = 112;
	Lf1 = 61.7134;
	Lc1 = 132.5449;

	Eigen::Matrix<double, 2, 1> lcal_0;
	Eigen::Matrix<double, 2, 1> d_lcal;

	lcal_0 << (qm10 - delta_qa * 1.55) * dr,\
			  (qs10 + delta_qa * 1.76) * dr;
	k = 0;
	F << Lm1 * cos(lcal_0(0,0))+Ls1 * cos(lcal_0(1,0))+Lf1 * cos(qf10*dr)+Lc1 * cos(qc1*dr),\
		 Lm1 * sin(lcal_0(0,0))+Ls1 * sin(lcal_0(1,0))+Lf1 * sin(qf10*dr)+Lc1 * sin(qc1*dr);
	while(F.norm()>1E-10){
		F << Lm1 * cos(lcal_0(0,0))+Ls1 * cos(lcal_0(1,0))+Lf1 * cos(qf10*dr)+Lc1 * cos(qc1*dr),\
		 	 Lm1 * sin(lcal_0(0,0))+Ls1 * sin(lcal_0(1,0))+Lf1 * sin(qf10*dr)+Lc1 * sin(qc1*dr);
		G << -Lm1 * sin(lcal_0(0,0)),\
		 	 -Ls1 * sin(lcal_0(1,0)),\
		 	  Lm1 * cos(lcal_0(0,0)),\
		 	  Ls1 * cos(lcal_0(1,0));

		d_lcal = -G.inverse() * F;
		lcal_0 += d_lcal;
		k++;
	   if (k >= 100)
	   break;
	}	 

	double qm20,qc20,qf20,qf2,delta_qk;
	qm20 = -122.7125;
	qc20 = -34.0795;
	qf20 = 167.8;
	qf2 = qf20 + q_passive(0,0)-qa0;
    delta_qk = q_passive(1,0);

	double Lm2,Ls2,Lf2,Lc2;
	Lm2 = 193;
	Ls2 = 222;
	Lf2 = 76.6775;
	Lc2 = 82;

	Eigen::Matrix<double, 2, 1> jcal_0;
	Eigen::Matrix<double, 2, 1> d_jcal;

	jcal_0 << (qm20 - delta_qk) * dr,\
			  (qc20 - delta_qk) * dr;
	k = 0;
	F << Lm2 * cos(jcal_0(0,0))+Ls2 * cos(q_passive(1,0)*dr)+Lf2 * cos(qf2*dr)+Lc2 * cos(jcal_0(1,0)),\
		 Lm2 * sin(jcal_0(0,0))+Ls2 * sin(q_passive(1,0)*dr)+Lf2 * sin(qf2*dr)+Lc2 * sin(jcal_0(1,0));
	while(F.norm()>1E-10){
		F << Lm2 * cos(jcal_0(0,0))+Ls2 * cos(q_passive(1,0)*dr)+Lf2 * cos(qf2*dr)+Lc2 * cos(jcal_0(1,0)),\
		 	 Lm2 * sin(jcal_0(0,0))+Ls2 * sin(q_passive(1,0)*dr)+Lf2 * sin(qf2*dr)+Lc2 * sin(jcal_0(1,0));
		G << -Lm2 * sin(jcal_0(0,0)),\
		 	 -Lc2 * sin(jcal_0(1,0)),\
		 	  Lm2 * cos(jcal_0(0,0)),\
		 	  Lc2 * cos(jcal_0(1,0));

		d_jcal = -G.inverse() * F;
		jcal_0 += d_jcal;
		k++;
	   if (k >= 100)
	   break;
	}	 
	rtY.ankle = lcal_0(0,0)*rd-qm10-delta_qa;
	rtY.knee = jcal_0(0,0)*rd-qm20;
	rtY.hip = 0;

	angle_a[i] = rtY.ankle;
	angle_k[i] = rtY.knee;
	angle_h[i] = rtY.hip;
	//cout << "angle_a[i]" << i << angle_a[i] << endl;
    //cout << "angle_k[i]" << i << angle_k[i] << endl;

}
	for (int vi = 0;vi <= 799;vi++){  
		t_real = data_delta;
		if (vi == 799 ){
			angular_a[vi] = 0;
			angular_k[vi] = 0;
			angular_h[vi] = 0;
		}
		else{
			angular_a[vi] = (angle_a[vi+1]-angle_a[vi])/t_real;
			angular_k[vi] = (angle_k[vi+1]-angle_k[vi])/t_real;
			angular_h[vi] = (angle_h[vi+1]-angle_h[vi])/t_real;
		}
	}
}
//关节控制发送 不用
void BaseController::send_joint_control(int i)
{
    BaseController::Joint_control joint_control_;
    joint_control_.left_ankle_vel = (int)(angle_a[i]*100);
    joint_control_.right_ankle_vel = (int)(angle_a[i]*100);
    joint_control_.left_knee_vel = (int)(angle_k[i]*100);
    joint_control_.right_knee_vel = (int)(angle_k[i]*100);
    joint_control_.left_hip_vel = (int) (angle_h[i]*100);
    joint_control_.right_hip_vel = (int)(angle_h[i]*100);
    //std::cout << "joint_control_.left_ankle_vel" << joint_control_.left_ankle_vel << std::endl;
    //std::cout << "joint_control_.right_ankle_vel" << joint_control_.right_ankle_vel << std::endl;
    //std::cout << "joint_control_.left_knee_vel" << joint_control_.left_knee_vel << std::endl;
    //std::cout << "joint_control_.right_knee_vel" << joint_control_.right_knee_vel << std::endl;
    //std::cout << "joint_control_.left_hip_vel" << joint_control_.left_hip_vel << std::endl;
    //std::cout << "joint_control_.right_hip_vel" << joint_control_.right_hip_vel << std::endl;
    joint_control_.left_ankle_angular = (int)(angular_a[i]*100);
    joint_control_.right_ankle_angular = (int)(angular_a[i]*100);
    joint_control_.left_knee_angular = (int)(angular_k[i]*100);
    joint_control_.right_knee_angular = (int)(angular_k[i]*100);
    joint_control_.left_hip_angular = (int)(angular_h[i]*100);
    joint_control_.right_hip_angular = (int)(angular_h[i]*100);
    //std::cout << "joint_control_.left_ankle_angular" << joint_control_.left_ankle_angular << std::endl;
    //std::cout << "joint_control_.right_ankle_angular" << joint_control_.right_ankle_angular << std::endl;
    //std::cout << "joint_control_.left_knee_angular" << joint_control_.left_knee_angular << std::endl;
    //std::cout << "joint_control_.right_knee_angular" << joint_control_.right_knee_angular << std::endl;
    //std::cout << "joint_control_.left_hip_angular" << joint_control_.left_hip_angular << std::endl;
    //std::cout << "joint_control_.right_hip_angular" << joint_control_.right_hip_angular << std::endl;
    control_vel[6] = joint_control_.left_hip_vel >> 16;
    control_vel[7] = joint_control_.left_hip_vel >> 8;
    //std::cout << control_vel[6] << std::endl;
    control_vel[8] = joint_control_.left_hip_angular >> 16;
    control_vel[9] = joint_control_.left_hip_angular >> 8;

    control_vel[10] = joint_control_.left_knee_vel >> 16;
    control_vel[11] = joint_control_.left_knee_vel >> 8;
    control_vel[12] = joint_control_.left_knee_angular >> 16;
    control_vel[13] = joint_control_.left_knee_angular >> 8;

    control_vel[14] = joint_control_.left_ankle_vel >> 16;
    control_vel[15] = joint_control_.left_ankle_vel >> 8;
    control_vel[16] = joint_control_.left_ankle_angular >> 16;
    control_vel[17] = joint_control_.left_ankle_angular >> 8;

    control_vel[26] = joint_control_.right_hip_vel >> 16;
    control_vel[27] = joint_control_.right_hip_vel >> 8;
    control_vel[28] = joint_control_.right_hip_angular >> 16;
    control_vel[29] = joint_control_.right_hip_angular >> 8;
    
    control_vel[30] = joint_control_.right_knee_vel >> 16;
    control_vel[31] = joint_control_.right_knee_vel >> 8;
    control_vel[32] = joint_control_.right_knee_angular >> 16;
    control_vel[33] = joint_control_.right_knee_angular >> 8;

    control_vel[34] = joint_control_.right_ankle_vel >> 16;
    control_vel[35] = joint_control_.right_ankle_vel >> 8;
    control_vel[36] = joint_control_.right_ankle_angular >> 16;
    control_vel[37] = joint_control_.right_ankle_angular >> 8;

    //serialManager -> send(control_vel, CONTROL_SIZE);
    //std::cout << "control_vel" << control_vel << std::endl;
    //printf("%s",control_vel);
}
// get_pos 校验 暂时不用
void BaseController::init_send_msgs()
{
    //get_pos[7]=xor_msgs(get_pos);
}
//校验
unsigned char BaseController::xor_msgs(unsigned char *msg)
{
    unsigned char check=msg[1];
    for(int i=2;i<7;i++)
        check=check ^ msg[i];
    return check;
}
// 发送轮子期望速度
void BaseController::sendVelocity()
{
    velocity_mutex_.lock();
   wheel_vel[5] = user_cmd_vel_.cmd_left_wheel;
   wheel_vel[6] = -user_cmd_vel_.cmd_right_wheel;
   velocity_mutex_.unlock();
   serialManager -> send(wheel_vel, COMMAND_SIZE);
   //std::cout << "sendVelocity" << std::endl;
}
//读取机器人底盘模型
void BaseController::setBaseModel(const std::string & param_addr)
{
    global_x -= BASE_MODEL_.Wheel_Center_X_Offset;
    global_y -= BASE_MODEL_.Wheel_Center_Y_Offset;
    YAML::Node doc = YAML::LoadFile(param_addr);
    try
    {
        BASE_MODEL_.Wheel_Diameter=doc["WheelDiameter"].as<double>();
        BASE_MODEL_.Wheel_Base = doc["WheelBase"].as<double>();
        BASE_MODEL_.Encoder_to_Distance = doc["EncoderToDistance"].as<double>();
        BASE_MODEL_.Wheel_Center_X_Offset = doc["WheelCenterXOffset"].as<double>();
        BASE_MODEL_.Wheel_Center_Y_Offset = doc["WheelCenterYOffset"].as<double>();
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("tagParam.yaml is invalid.");
    }
    global_x+=BASE_MODEL_.Wheel_Center_X_Offset;
    global_y+=BASE_MODEL_.Wheel_Center_Y_Offset;
}
//下位机数据发送
void BaseController::sendtimerCallback(const ros::TimerEvent &e)
{
    static int encoder_counter=1;

    //sendcommand
	if(encoder_counter%2 == 1)//100hz
    	//sendCommand();
        serialManager -> send(get_JointEncoder,COMMAND_SIZE);
        LeftForceSensor -> send(get_force,4);
        RightForceSensor -> send(get_force,4);
	if(encoder_counter%4==0)   //50hz
        serialManager -> send(get_pos,COMMAND_SIZE);
    if(encoder_counter % 8 ==2) //25hz
        sendCommand();
    if(encoder_counter % 8 == 6)//25hz
        sendVelocity();
	/*if(encoder_counter == 236&&send_imu_)
    {
		//15 hz
	    imu_mutex_.lock();
        serialManager -> send(imu_Pitch,10);
        imu_mutex_.unlock();
    }*/
	encoder_counter++;
	encoder_counter = encoder_counter == TIMER_SPAN_RATE_*2+1 ? 1 : encoder_counter;
}
//下位机数据接收
void BaseController::readtimerCallback(const ros::TimerEvent &e)
{
    NaviSerialManager::ReadResult self_results{serialManager->getReadResult()};
    encoder_pre = encoder_after;
    if(self_results.read_bytes>=COMMAND_SIZE)
    {
        int k = 8;
        for (int i = 0; i < self_results.read_bytes; i += k)
        {
            if(self_results.read_result[i] == COMMAND_HEAD)
            {
                k = 8;
                memcpy(message_odom_,&self_results.read_result[i],COMMAND_SIZE);
                parsingMsg();
            }
            else if(self_results.read_result[i] == ENCODER_HEAD)
            {
                k = 12;
                memcpy(message_encoder_,&self_results.read_result[i],ENCODER_SIZE);
                parsingEncoder();
            }
            else
            {
                ROS_WARN_STREAM(" THE DATA OF ODOM/ENCODER IS ERROR");  
            }
        }
          
        if(right_updated&&left_updated)
        {
            ENCODER_.interval=(encoder_after-encoder_pre).toSec();
            odom_parsing();
            right_updated=false;
            left_updated=false;
        }
    }
    else
        memset(message_odom_, 0, COMMAND_SIZE);
        memset(message_encoder_,0,ENCODER_SIZE);
    //publish the encoder when have.
        if(ENCODER_.interval!=0)
        {
            if(ENCODER_.interval>20.0/TIMER_SPAN_RATE_||ENCODER_.interval<0.0)
            {
                if(!ENCODER_.encoderWrong)
                 encoder_stop=ros::Time::now();
                ENCODER_.encoderWrong=true;
                ROS_ERROR_STREAM("Encoder once passed 5 frames");
            }
            else
            {   
                if(ENCODER_.encoderWrong)
                {
                    if((encoder_pre-encoder_stop).toSec()>=1.0)
                        ENCODER_.encoderWrong=false;
                }
            }
            basecontrol::WheelStatus wheelStatus{};
            wheelStatus.right_encoder=ENCODER_.right_encoder;
            wheelStatus.left_encoder = ENCODER_.left_encoder;
            wheelStatus.encoderWrong = ENCODER_.encoderWrong;
            wheelStatus.interval = ENCODER_.interval;
            wheel_status_pub.publish(wheelStatus);
        }
}
//发送控制指令
void BaseController::sendCommand( double parameter)
{
    command_mutex_.lock();
    switch (user_command_)
    {
		case IMU:
			send_imu_ = !send_imu_;
        case GET_POSE:
            //serialManager->send(get_pos,COMMAND_SIZE);
            break;
        case MOVEFORWARD:
            if (NaviMode)
               // serialManager -> send(move_forward,COMMAND_SIZE);
            
                ROS_WARN(" ControlMode ");
            break;
        case MOVEBACK:
            if (NaviMode) 
               // serialManager -> send(move_back,COMMAND_SIZE);
            
                ROS_WARN(" ControlMode ");
            break;
        case TURNLEFT:
            if (NaviMode) 
               // serialManager -> send(turn_left,COMMAND_SIZE);
            
                ROS_WARN(" ControlMode ");
            break;
        case TURNRIGHT:
            if (NaviMode) 
              //  serialManager -> send(turn_right,COMMAND_SIZE);
            
                ROS_WARN(" ControlMode ");
            break;
        case STOP:
            serialManager -> send(stop_smooth,COMMAND_SIZE);
            break;
        case RESET:
			std::cout<<"reset here"<<std::endl;
            serialManager -> send(joint_reset,COMMAND_SIZE);
            break;
        case INIT:
            serialManager -> send(joint_init,COMMAND_SIZE);
            break;
        case SQUAT:
            serialManager -> send(squat,COMMAND_SIZE);
			std::cout << "squat 1" << std::endl;
            break;
        case STAND:
            serialManager -> send(stand,COMMAND_SIZE);
			std::cout << "stand 1" << std::endl;
            break;
        case KNEECUT:
            serialManager -> send(left_knee_cut,COMMAND_SIZE);
			std::cout << "kneecut 1" << std::endl;
            usleep(500000);
            serialManager -> send(right_knee_cut,COMMAND_SIZE);
			break;
        case ANKLECUT://clear message
            //serialManager -> send(left_ankle_cut,COMMAND_SIZE);
			//std::cout << "anklecut 1" << std::endl;	
            usleep(500000);
            serialManager -> send(clear_message,COMMAND_SIZE);
            break;
        default:
            break;
    }
    user_command_ = Command::DEFAULT;
    command_mutex_.unlock();
}

void BaseController::passCommand(Command user_command)
{
    command_mutex_.lock();
    user_command_ = user_command;
    command_mutex_.unlock();
}
//码盘解算
int BaseController::parsingMsg()
{
    if(0x35!=message_odom_[0])
    {
        memset(message_odom_,0,COMMAND_SIZE);
        return -1;
    }
    else
    {
        switch (message_odom_[1])
        {
            case 0x31:
                /*preserved*/
                break;
            case 0x21:
                /*poistion of right wheel*/
                if(0x13==message_odom_[2])
                {
                    //right encodisk parsing
                    char* pchar = (char*)&ENCODER_.right_encoder;
                    *(pchar+3) = message_odom_[3];
                    *(pchar+2) = message_odom_[4];
                    *(pchar+1) = message_odom_[5];
                    *(pchar+0) = message_odom_[6];

                    if (std::abs(ENCODER_.right_encoder) > INT_MAX - 1000)
                        ENCODER_.right_encoder = 0;

                    //make sure the consistency of left and right
                    right_updated= true;
                    encoder_after = ros::Time::now();
                }
                break;
            case 0x11:
                /*position of left wheel*/
                if(0x13==message_odom_[2])
                {
                    char* pchar = (char*)&ENCODER_.left_encoder;
                    *(pchar+3) = message_odom_[3];
                    *(pchar+2) = message_odom_[4];
                    *(pchar+1) = message_odom_[5];
                    *(pchar+0) = message_odom_[6];

                    //按旧小车，向前进时，左轮码盘为负，要乘负一
                    ENCODER_.left_encoder *=-1;

                    if (std::abs(ENCODER_.left_encoder) > INT_MAX - 1000)
                        ENCODER_.left_encoder = 0;
                    //make sure the consistency of left and right
                    left_updated=true;
                    encoder_after = ros::Time::now();
                }
                break;
			default:
                break;
        }
    }
    return 0;
}

int BaseController::parsingEncoder()
{

}
//里程计计算
void BaseController::odom_parsing()
{
    static int right_encoder_pre{ENCODER_.right_encoder},left_encoder_pre{ENCODER_.left_encoder};
    static ros::Time last_time{ros::Time::now()};

    int right_delta{ENCODER_.right_encoder - right_encoder_pre};
    int left_delta{ENCODER_.left_encoder - left_encoder_pre};


    double dt = (ros::Time::now()-last_time).toSec();
    double right_distance{},left_distance{};
    double theta{};

    // the robot is not moving
    if(abs(left_delta)<2&&abs(right_delta)<2)
    {
        linear_velocity_=0;
        angular_velocity_=0;
        //global position don't change

        //update value
        right_encoder_pre = ENCODER_.right_encoder;
        left_encoder_pre = ENCODER_.left_encoder;
        last_time = ros::Time::now();
        return;
    }

    //in case of wrong data poisoning;
    //Noted, this is because  value bigger than the int limit;
    if(abs(left_delta)>1000||abs(right_delta)>1000)
    {
        //we hope to keep the same value of last time
        //so just update value and return;

        right_encoder_pre = ENCODER_.right_encoder;
        left_encoder_pre = ENCODER_.left_encoder;
        last_time = ros::Time::now();
        return ;
    }

    //in case of EMERGENCY button pressed
    if(ENCODER_.right_encoder==0&&ENCODER_.left_encoder==0&&right_encoder_pre!=0&&left_encoder_pre!=0)
    {
        //we hope to keep the same value of last time
        //so just update value and return;

        right_encoder_pre = ENCODER_.right_encoder;
        left_encoder_pre = ENCODER_.left_encoder;
        last_time = ros::Time::now();
        return ;
    }


    right_distance = M_PI * BASE_MODEL_.Wheel_Diameter * right_delta / BASE_MODEL_.Encoder_to_Distance;//mm
    left_distance  = M_PI * BASE_MODEL_.Wheel_Diameter * left_delta / BASE_MODEL_.Encoder_to_Distance;//mm
    theta = (right_distance-left_distance)/BASE_MODEL_.Wheel_Base;
    if(left_delta==right_delta)
    {
        //moving forward
        double vertical_robot{((right_distance+left_distance)/2.0)};//mm
        vertical_robot /= 1000.0; //mm to m;
        double horizontal_robot{};//m

        global_x += vertical_robot*cos(global_theta) - horizontal_robot*sin(global_theta);//m
        global_y += horizontal_robot*cos(global_theta) + vertical_robot*sin(global_theta);//m
        global_theta +=theta;

        linear_velocity_ = (right_distance+left_distance)/(2000.0*dt);
        angular_velocity_ = theta/dt;
    }
    else
    {
        //turing round
        double turning_radius;
        turning_radius =(BASE_MODEL_.Wheel_Base*(right_distance+left_distance))/(2.0*(right_distance-left_distance));//m

        double vertical_robot{turning_radius*sin(theta)};//mm
        double horizontal_robot{turning_radius*(1-cos(theta))};//mm

        vertical_robot /= 1000.0;//m
        horizontal_robot /= 1000.0;//m

        global_x += vertical_robot*cos(global_theta) - horizontal_robot*sin(global_theta);//m
        global_y += horizontal_robot*cos(global_theta) + vertical_robot*sin(global_theta);//m
        global_theta +=theta;

        linear_velocity_ = (right_distance+left_distance)/(2000.0*dt);
        angular_velocity_ = theta/dt;
    }

    //update value
    right_encoder_pre = ENCODER_.right_encoder;
    left_encoder_pre = ENCODER_.left_encoder;
    last_time = ros::Time::now();
}
//里程计发布
void BaseController::odom_publish_timer_callback(const ros::TimerEvent &e)
{
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(global_theta);

    odom.header.stamp = odom_trans.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_trans.header.frame_id = ODOM_FRAME_;
    odom.child_frame_id = odom_trans.child_frame_id = BASE_FOOT_PRINT_;

    odom.pose.pose.position.x = odom_trans.transform.translation.x =global_x;
    odom.pose.pose.position.y = odom_trans.transform.translation.y =global_y;
    odom.pose.pose.position.z = odom_trans.transform.translation.z =0.0;
    odom.pose.pose.orientation = odom_trans.transform.rotation = odom_quat;

    odom.pose.covariance=	{1e-3,0,0,0,0,0,
                              0,1e-3,0,0,0,0,
                              0,0,1e6,0,0,0,
                              0,0,0,1e6,0,0,
                              0,0,0,0,1e6,0,
                              0,0,0,0,0,1e3};

    odom.twist.twist.linear.x = linear_velocity_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = angular_velocity_;

    odom.twist.covariance={1e-3,0,0,0,0,0,
                           0,1e-3,0,0,0,0,
                           0,0,1e6,0,0,0,
                           0,0,0,1e6,0,0,
                           0,0,0,0,1e6,0,
                           0,0,0,0,0,1e3};

    odom_raw_pub.publish(odom);

    if(publish_tf_)
        broad_caster.sendTransform(odom_trans);
}
//多IMU数据发送 后期取消
void BaseController::MultiImuCallback (const sensor_msgs::Imu & robot_Pitch)
{
    imu_mutex_.lock();
	 tf::Quaternion quat;
     tf::quaternionMsgToTF(robot_Pitch.orientation,quat);
    double roll, pitch,yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  
    tou = pitch* 1000;
	tou_1 = roll * 1000;
	//std::cout << tou << std::endl;
    imu_Pitch[1] = (tou >> 8) & 0xff;
	imu_Pitch[2] = tou & 0xff;
	//imu_Pitch[3] = (zuo >> 8) & 0xff ;
    //imu_Pitch[4] = zuo & 0xff ;
	//imu_Pitch[5] = (you >> 8) & 0xff;
	//imu_Pitch[6] = you & 0xff;
	 imu_Pitch[7] = (tou_1 >> 8) & 0xff;
	 imu_Pitch[8] = tou_1 & 0xff;
	//imu_Pitch[1] = -imu_Pitch[1];
	//imu_Pitch[2] = -imu_Pitch[2];
	imu_mutex_.unlock();
}
