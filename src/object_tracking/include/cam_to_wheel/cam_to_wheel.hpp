#ifndef _CAM_TO_WHEEL_
#define _CAM_TO_WHEEL_

#include <stdlib.h>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <object_tracking/wheel_msg.h>
#include "pid.hpp"

class Cam_to_Wheel
{
private:
    ros::NodeHandle nh_;

    image_transport::ImageTransport img_trans_;
    image_transport::Subscriber sub_;

    ros::Publisher pub_;

    const sensor_msgs::ImageConstPtr img_msg_;
    object_tracking::wheel_msg wheel_msg_;

    cv::Mat frame_;

    double pixel_dist_x_;

    PID pid_;

public:
    Cam_to_Wheel(ros::NodeHandle &nh, PID &pid);
    ~Cam_to_Wheel();

    bool Recieve();
    bool ImageProcess();
    bool PID_Control();
    bool Publish();

    void RecvCallback(const sensor_msgs::ImageConstPtr &img_msg);
};

struct HSV_Bound
{
    cv::Scalar upper_green = cv::Scalar(77, 255, 255);
    cv::Scalar lower_green = cv::Scalar(35, 43, 23);
    cv::Scalar upper_blue = cv::Scalar(99, 255, 255);
    cv::Scalar lower_blue = cv::Scalar(78, 43, 46);
    cv::Scalar upper_violet = cv::Scalar(180, 255, 255);
    cv::Scalar lower_violet = cv::Scalar(156, 43, 46);
    cv::Scalar upper_orange = cv::Scalar(25, 255, 255);
    cv::Scalar lower_orange = cv::Scalar(0, 90, 90);
}HSV;

#endif