#include "basecontrol/ActionLib.h"
#include "basecontrol/basecontrol.h"

using namespace cv;
ActionLib::ActionLib(std::string serial_addr)
{
    action_flag_ = Default;
    DropPrevention_Sub = nh_.subscribe("/camera/depth/image_rect_raw",1000,&ActionLib::DropPreventionCallback,this);
}

ActionLib::~ActionLib()
{

}

void ActionLib::DropPreventionCallback(const sensor_msgs::Image & image)
{
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
                depth_value += img.at<uint16_t>(m,n);
                k += 1;
            }
    }
    depth_value = depth_value/k;
    latest_depth.data=depth_value;
    //cout << "depth_value is " << depth_value << endl;
    shuiping_depth.publish(latest_depth);
    
    dp_change = depth_value - dp1;
    dp_change = abs(dp_change);
    if(dp_change > 500)
        {
            std::cout << "We should stop!" << std::endl;
            std::cout << "difference is " << dp_change << std::endl;
            action_flag_ = DropPrevention;
        } 
}
