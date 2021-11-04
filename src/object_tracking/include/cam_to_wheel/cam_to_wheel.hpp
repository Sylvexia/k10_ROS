#ifndef _CAM_TO_WHEEL_
#define _CAM_TO_WHEEL_

#include <ros/ros.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <object_tracking/wheel_msg.h>

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
    cv::Mat gray_;
    cv::Mat blur_;
    cv::Mat canny_;
    cv::Mat layout_;

    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierachy_;

public:
    Cam_to_Wheel(ros::NodeHandle &nh);
    ~Cam_to_Wheel();

    bool Recieve();
    bool ImageProcess();
    bool Publish();

    void RecvCallback(const sensor_msgs::ImageConstPtr &img_msg);
};

#endif