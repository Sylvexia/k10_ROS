#ifndef _CAM_TO_WHEEL_
#define _CAM_TO_WHEEL_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

class Cam_to_Wheel
{
private:
    ros::NodeHandle nh_;

    image_transport::ImageTransport img_trans_;
    image_transport::Subscriber sub_;

    const sensor_msgs::ImageConstPtr img_msg_;

public:
    Cam_to_Wheel(ros::NodeHandle &nh);
    ~Cam_to_Wheel();

    bool Recieve();
    bool ProcessData();
    bool Control();

    void RecvCallback(const sensor_msgs::ImageConstPtr &img_msg);
};

#endif