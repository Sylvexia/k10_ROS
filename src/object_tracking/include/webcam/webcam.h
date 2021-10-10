#ifndef _WEBCAM_H_
#define _WEBCAM_H_

#include <ros/ros.h>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>

class WebCamera
{

private:
    ros::NodeHandle nodehandle_;

    image_transport::ImageTransport image_transport_;
    image_transport::Publisher publisher_;

    cv_bridge::CvImage ImageTransport(cv::Mat &image);

public:
    WebCamera(ros::NodeHandle &nh);
    ~WebCamera();

    bool Publish();
};

#endif