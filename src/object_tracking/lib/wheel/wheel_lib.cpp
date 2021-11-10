#include "wheel/wheel.hpp"

Wheel::Wheel(ros::NodeHandle &nh, PID &pid)
    : nh_(nh), pid_(pid)
{
    ROS_INFO("\nWheel Class Constructed");
}

Wheel::~Wheel()
{
    ROS_INFO("\nWheel Class Destructed");
}

bool Wheel::Run()
{
    sub_ = nh_.subscribe("cam/wheel", 500, &Wheel::RecvCallback, this);
    pub_ = nh_.advertise<object_tracking::wheel_msg>("wheel", 50);

    pid_.init();
    return true;
}

bool Wheel::Publish()
{
    wheel_msg_.left = pid_.pidCtrl(pixel_dist_x_, 0);
    wheel_msg_.right = -(pid_.pidCtrl(pixel_dist_x_, 0));
    ROS_INFO("right: [%d]", pixel_dist_x_);
    pub_.publish(wheel_msg_);
    return true;
}

void Wheel::RecvCallback(object_tracking::image_dist_msg dist_msg)
{
    pixel_dist_x_ = dist_msg.pixel_dist_x;
    Publish();

    return;
}