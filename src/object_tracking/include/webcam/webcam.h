#ifndef _WEBCAM_H_
#define _WEBCAM_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

class WebCamera
{

private:
    ros::NodeHandle nh_;

    image_transport::ImageTransport image_transport_;
    image_transport::Publisher publisher_;

    cv::VideoCapture cap_;

    cv::Mat frame_;
    cv::Mat resized_frame_;

    sensor_msgs::ImagePtr img_msg_;

public:
    WebCamera(ros::NodeHandle &nh);
    ~WebCamera();

    bool Capture();
    bool Publish();
    bool Imshow(int x, int y);
};

#endif