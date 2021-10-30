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
        frame_ = cv_bridge::toCvShare(img_msg_, "bgr8")->image;
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg_->encoding.c_str());
    }

    ImageProcess();

    Publish();

    return;
}

bool Cam_to_Wheel::Recieve()
{
    sub_ = img_trans_.subscribe("camera/image", 1, &Cam_to_Wheel::RecvCallback, this);
    return true;
}

bool Cam_to_Wheel::ImageProcess()
{
    cv::cvtColor(frame_, gray_, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray_, blur_, cv::Size2d(11, 11), 0);
    cv::Canny(blur_, canny_, 20, 160);
    cv::findContours(canny_, contours_, hierachy_, cv::RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    layout_=cv::Mat::zeros(canny_.size(),CV_8SC3);

    for (size_t i = 0; i < contours_.size(); i++)
        cv::drawContours(layout_, contours_, i, (0, 255, 0), 2, cv::LINE_8, hierachy_);

    cv::imshow("layout", blur_);

    return true;
}

bool Cam_to_Wheel::Publish()
{
    return true;
}