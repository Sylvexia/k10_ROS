#include "cam_to_wheel/cam_to_wheel.hpp"

Cam_to_Wheel::Cam_to_Wheel(ros::NodeHandle &nh, PID &pid)
    : nh_(nh), img_trans_(nh), pid_(pid), pixel_dist_x_(0.0)
{
    ROS_INFO("\nCam_to_Wheel Class Constructed\n");
}

Cam_to_Wheel::~Cam_to_Wheel()
{
    ROS_INFO("\nCam_to_Wheel Class Destructed\n");
}

bool Cam_to_Wheel::Recieve()
{
    sub_ = img_trans_.subscribe("camera/image", 1, &Cam_to_Wheel::RecvCallback, this);
    pub_ = nh_.advertise<object_tracking::wheel_msg>("wheel", 500);

    pid_.init();
    return true;
}

void Cam_to_Wheel::RecvCallback(const sensor_msgs::ImageConstPtr &img_msg_)
{
    try
    {
        frame_ = cv_bridge::toCvShare(img_msg_, "bgr8")->image;
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg_->encoding.c_str());
    }

    ImageProcess();
    PID_Control();
    Publish();

    return;
}

bool Cam_to_Wheel::ImageProcess()
{
    cv::Scalar bgr_white = cv::Scalar(255, 255, 255);
    cv::Scalar bgr_red = cv::Scalar(0, 0, 255);
    cv::Scalar bgr_green = cv::Scalar(0, 255, 0);
    cv::Scalar bgr_blue = cv::Scalar(255, 255, 0);

    cv::Point2d anchor = cv::Point2d(-1, -1);

    cv::Scalar hsv_upper_set = HSV.upper_green;//uno card
    cv::Scalar hsv_lower_set = HSV.lower_green;

    cv::Mat blur;
    cv::Mat hsv;
    cv::Mat mask;
    cv::Mat opening;
    cv::Mat erosion;
    cv::Mat dilate;
    cv::Mat canny;
    cv::Mat layout = cv::Mat::zeros(frame_.size(), CV_8SC3);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;

    //process
    cv::GaussianBlur(frame_, blur, cv::Size2d(11, 11), 0);
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_lower_set, hsv_upper_set, mask);
    cv::morphologyEx(mask, opening, cv::MORPH_OPEN, std::vector<int>(5, 5), anchor, 2);
    cv::erode(opening, erosion, std::vector<int>(5, 5), anchor, 2);
    cv::Canny(erosion, canny, 20, 160);
    cv::dilate(canny, dilate, std::vector<int>(5, 5), anchor, 2);
    cv::findContours(canny, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> minRect(contours.size());
    cv::Point2f rect_points[4];
    cv::Point2f centers[contours.size()];
    cv::Point2f centroid(0.f, 0.f);

    for (size_t i = 0; i < contours.size(); ++i)
    {
        minRect[i] = cv::minAreaRect(contours[i]);
        minRect[i].points(rect_points);

        centers[i] = minRect[i].center;
        centroid.x += centers[i].x;
        centroid.y += centers[i].y;

        cv::drawContours(layout, contours, i, bgr_white, 2, cv::LINE_8, hierachy);
        cv::circle(layout, centers[i], 4, bgr_green);

        for (int j = 0; j < 4; ++j)
            cv::line(layout, rect_points[j], rect_points[(j + 1) % 4], bgr_red, 5);
    }
    centroid.x = centroid.x / contours.size();
    centroid.y = centroid.y / contours.size();
    cv::circle(layout, centroid, 16, bgr_blue, -1);

    pixel_dist_x_ = (frame_.cols / 2) - centroid.x;

    cv::imshow("mask", mask);
    cv::imshow("canny", canny);
    cv::imshow("dilate", dilate);
    cv::imshow("layout", layout);

    return true;
}

bool Cam_to_Wheel::PID_Control()
{
    ROS_INFO("Pixel_Distance_X[%f]", pixel_dist_x_);
    ROS_INFO("Control Variable[%f]", pid_.pidCtrl(pixel_dist_x_, 0.0));

    return true;
}

bool Cam_to_Wheel::Publish()
{
    wheel_msg_.left = 69;
    wheel_msg_.right = 96;

    wheel_msg_.right++;
    wheel_msg_.left++;

    //pub_.publish(wheel_msg_);

    return true;
}