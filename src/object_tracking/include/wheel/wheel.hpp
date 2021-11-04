#ifndef _WHEEL_H_
#define _WHEEL_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <object_tracking/wheel_msg.h>

class Wheel
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

public:
    Wheel(ros::NodeHandle &nh);
    ~Wheel();

    bool Run();

    void RecvCallback(object_tracking::wheel_msg wheel_msg);
};

#endif