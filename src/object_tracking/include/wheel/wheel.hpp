#ifndef _WHEEL_H_
#define _WHEEL_H_

#include <ros/ros.h>
#include <object_tracking/wheel_msg.h>
#include <object_tracking/image_dist_msg.h>

#include "pid.hpp"

//wheel control for two wheel motor

class Wheel
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    object_tracking::wheel_msg wheel_msg_;
    uint32_t pixel_dist_x_;

    PID pid_;

public:
    Wheel(ros::NodeHandle &nh, PID &pid);
    ~Wheel();

    bool Run();

    void RecvCallback(object_tracking::image_dist_msg dist_msg);

    bool Publish();
};

#endif