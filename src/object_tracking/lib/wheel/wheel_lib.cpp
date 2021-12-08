#include "wheel/wheel.hpp"

Wheel::Wheel()
{
    ROS_INFO("Default Wheel Constructor");
}

Wheel::Wheel(ros::NodeHandle &nh, PID &pid)
    : nh_(nh), pid_(pid), pid_value_(0.0)
{
    ROS_INFO("\nWheel PID Class Constructed");
}

Wheel::Wheel(ros::NodeHandle &nh)
    : nh_(nh)
{
    ROS_INFO("\nWheel Ctrl Class Constructed");
}

Wheel::~Wheel()
{
    ROS_INFO("\nWheel Class Destructed");
}

bool Wheel::Run_PID()
{
    pid_.init();

    sub_ = nh_.subscribe("cam/wheel", 500, &Wheel::Run_PID_Callback, this);
    pub_ = nh_.advertise<object_tracking::wheel_msg>("wheel", 50);

    return true;
}

bool Wheel::KeyboardCtrl()
{
    return true;
}

bool Wheel::Publish()
{
    pid_value_ = pid_.pidCtrl(pixel_dist_x_, 0.01);

    wheel_msg_.left = int(pid_value_);
    wheel_msg_.right = int(-(pid_value_));

    pub_.publish(wheel_msg_);

    return true;
}

void Wheel::Run_PID_Callback(object_tracking::image_dist_msg img_dist_msg)
{
    pixel_dist_x_ = img_dist_msg.pixel_dist_x;
    Publish();

    return;
}