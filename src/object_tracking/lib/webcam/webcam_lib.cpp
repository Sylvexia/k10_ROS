#include "webcam/webcam.h"

WebCamera::WebCamera(ros::NodeHandle &nh)
    : image_transport_(nh), nh_(nh), cap_(0)
{
    publisher_ = image_transport_.advertise("camera/image", 1);

    if (!cap_.isOpened())
        ROS_ERROR("\ncamera not opened\n");
    ROS_INFO("\nClass Webcam constructed\n");
}

WebCamera::~WebCamera()
{
    ROS_INFO("Class Webcam Destructed");
}

bool WebCamera::Capture()
{
    cap_ >> frame_;

    return true;
}

bool WebCamera::Publish()
{
    if (this->frame_.empty())
    {
        ROS_INFO("empty");
        return false;
    }

    img_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();
    publisher_.publish(img_msg_);
    return true;
}

bool WebCamera::Imshow(int x, int y)
{
    //cv::resize(frame_, resized_frame_, cv::Size(640, 480));
    ROS_INFO("%d, %d", frame_.rows, frame_.cols);
    //ROS_INFO("%d", resized_frame_.rows);
    cv::imshow("resize_frame", frame_);
    return true;
}