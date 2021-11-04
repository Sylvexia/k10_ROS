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

bool Cam_to_Wheel::Recieve()
{
    sub_ = img_trans_.subscribe("camera/image", 1, &Cam_to_Wheel::RecvCallback, this);
    pub_ = nh_.advertise<object_tracking::wheel_msg>("wheel", 500);
    return true;
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

bool Cam_to_Wheel::ImageProcess()
{
    cv::Scalar white = cv::Scalar(255, 255, 255);
    cv::Scalar red = cv::Scalar(0, 0, 255);

    //process
    cv::cvtColor(frame_, gray_, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray_, blur_, cv::Size2d(11, 11), 0);
    cv::Canny(blur_, canny_, 20, 160);
    cv::findContours(canny_, contours_, hierachy_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> minRect(contours_.size());
    cv::Point2f rect_points[4];

    layout_ = cv::Mat::zeros(frame_.size(), CV_8SC3);

    for (size_t i = 0; i < contours_.size(); ++i)
    {
        minRect[i] = cv::minAreaRect(contours_[i]);
        minRect[i].points(rect_points);

        cv::drawContours(layout_, contours_, i, white, 2, cv::LINE_8, hierachy_);
        for (int j = 0; j < 4; ++j)
            cv::line(layout_, rect_points[j], rect_points[(j + 1) % 4], red, 5);
    }

    cv::imshow("layout", layout_);

    return true;
}

bool Cam_to_Wheel::Publish()
{
    wheel_msg_.left = 69;
    wheel_msg_.right = 96;

    wheel_msg_.right++;
    wheel_msg_.left++;

    pub_.publish(wheel_msg_);

    return true;
}