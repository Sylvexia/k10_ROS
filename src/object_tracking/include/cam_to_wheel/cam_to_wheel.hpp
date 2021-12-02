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

#include <object_tracking/image_dist_msg.h>
#include "pid.hpp"

class Cam_to_Wheel
{
private:
    ros::NodeHandle nh_;

    image_transport::ImageTransport img_trans_;
    image_transport::Subscriber sub_;

    ros::Publisher pub_;

    const sensor_msgs::ImageConstPtr img_msg_;
    object_tracking::image_dist_msg dist_msg_;

    cv::Scalar hsv_upper_bound_;
    cv::Scalar hsv_lower_bound_;

    cv::Mat frame_;

    cv::Mat blur_;
    cv::Mat hsv_;
    cv::Mat mask_;
    cv::Mat opening_;
    cv::Mat closing_;
    cv::Mat erosion_;
    cv::Mat dilate_;
    cv::Mat canny_;
    cv::Mat layout_;

    cv::Mat pre_proc_;

    double pixel_dist_x_;
    double pixel_dist_y_;

public:
    Cam_to_Wheel(ros::NodeHandle &nh);
    ~Cam_to_Wheel();

    bool Recieve();
    bool ImageProcess();
    bool Publish();

    cv::Mat ImagePreProc(cv::Mat input);

    bool ImageWeightedCentroid();
    bool ImageLargest();

    void RecvCallback(const sensor_msgs::ImageConstPtr &img_msg);
};

struct HSV_Bound
{
    cv::Scalar upper_green = cv::Scalar(60, 255, 255);
    cv::Scalar lower_green = cv::Scalar(40, 21, 21);
    cv::Scalar upper_blue = cv::Scalar(117, 255, 255);
    cv::Scalar lower_blue = cv::Scalar(97, 43, 46);
    cv::Scalar upper_purple = cv::Scalar(146, 255, 255);
    cv::Scalar lower_purple = cv::Scalar(126, 43, 46);
    cv::Scalar upper_violet = cv::Scalar(173, 255, 255);
    cv::Scalar lower_violet = cv::Scalar(156, 43, 46);
    cv::Scalar upper_orange = cv::Scalar(25, 255, 255);
    cv::Scalar lower_orange = cv::Scalar(0, 43, 46);
    cv::Scalar upper_red = cv::Scalar(6, 255, 255);
    cv::Scalar lower_red = cv::Scalar(0, 43, 46);
    cv::Scalar upper_yellow = cv::Scalar(33, 255, 255);
    cv::Scalar lower_yellow = cv::Scalar(21, 43, 46);
} HSV;

struct BGR_Color
{
    cv::Scalar white = cv::Scalar(255, 255, 255);
    cv::Scalar red = cv::Scalar(0, 0, 255);
    cv::Scalar green = cv::Scalar(0, 255, 0);
    cv::Scalar blue = cv::Scalar(255, 255, 0);
} BGR;

#endif