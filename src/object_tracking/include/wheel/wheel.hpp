#ifndef _WHEEL_H_
#define _WHEEL_H_

#include <ros/ros.h>
#include <object_tracking/wheel_msg.h>
#include <object_tracking/image_dist_msg.h>
#include <iostream>

#include "conio.h"
#include "pid.hpp"

//wheel control for two wheel motor

class Wheel
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    object_tracking::wheel_msg wheel_msg_;
    double pixel_dist_x_;

    PID pid_;
    
    double pid_value_;

public:
    Wheel(ros::NodeHandle &nh, PID &pid);
    Wheel(ros::NodeHandle &nh);
    ~Wheel();

    bool Run_PID();
    bool KeyboardCtrl();
    bool Publish();

    void Run_PID_Callback(object_tracking::image_dist_msg img_dist_msg);
};

#endif