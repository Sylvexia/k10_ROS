#include "wheel/wheel.hpp"

Wheel::Wheel(ros::NodeHandle &nh)
    : nh_(nh)
{
    ROS_INFO("\nWheel Class Constructed");
}

Wheel::~Wheel()
{
    ROS_INFO("\nWheel Class Destructed");
}

bool Wheel::Run()
{
    sub_ = nh_.subscribe("wheel", 500, &Wheel::RecvCallback,this);
    return true;
}

void Wheel::RecvCallback(object_tracking::wheel_msg wheel_msg)
{
    ROS_INFO("right: [%d]",wheel_msg.right);
    ROS_INFO("left: [%d]",wheel_msg.left);
    return;
}