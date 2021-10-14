#include "cam_to_wheel/cam_to_wheel.hpp"

Cam_to_Wheel::Cam_to_Wheel(ros::NodeHandle &nh)
    : nh_(nh), img_trans_(nh)
{
    ROS_INFO("\nCam_to_Wheel Class Constructed\n");
}

Cam_to_Wheel::~Cam_to_Wheel()
{
    ROS_INFO("\nCam_to_Wheel Class Destructed\n");
}

void Cam_to_Wheel::RecvCallback(const sensor_msgs::ImageConstPtr &img_msg_)
{
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(img_msg_, "bgr8")->image);
        cv::waitKey(30);
        ROS_INFO("recieved image");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg_->encoding.c_str());
    }
    return;
}

bool Cam_to_Wheel::Recieve()
{
    sub_ = img_trans_.subscribe("camera/image", 1, &Cam_to_Wheel::RecvCallback, this);
    return true;
}