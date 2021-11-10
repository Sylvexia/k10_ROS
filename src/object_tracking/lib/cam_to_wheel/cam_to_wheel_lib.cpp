#include "cam_to_wheel/cam_to_wheel.hpp"

Cam_to_Wheel::Cam_to_Wheel(ros::NodeHandle &nh)
    : nh_(nh), img_trans_(nh), pixel_dist_x_(0.0)
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
    pub_ = nh_.advertise<object_tracking::image_dist_msg>("cam/wheel", 500);

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
    Publish();

    return;
}

bool Cam_to_Wheel::ImageProcess()
{
    cv::Point2d anchor = cv::Point2d(-1, -1);

    cv::Scalar hsv_upper_set = HSV.upper_green;
    cv::Scalar hsv_lower_set = HSV.lower_green;

    cv::Mat blur;
    cv::Mat hsv;
    cv::Mat mask;
    cv::Mat opening;
    cv::Mat closing;
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
    cv::erode(mask, erosion, std::vector<int>(3, 3), anchor, 6);
    cv::dilate(erosion, dilate, std::vector<int>(3, 3), anchor, 6);
    cv::morphologyEx(dilate, opening, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size2d(7, 7)), anchor, 1);
    cv::Canny(opening, canny, 40, 160);
    cv::findContours(canny, contours, hierachy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> minRect;
    cv::Point2f rect_points[4];
    cv::Point2f centroid(0.f, 0.f);

    minRect.reserve(contours.size());

    for (size_t i = 0; i < contours.size(); ++i)
    {
        if (cv::contourArea(contours[i]) > 100)
        {
            minRect.emplace_back(cv::minAreaRect(contours[i]));
            cv::drawContours(layout, contours, i, BGR.white, -1, cv::LINE_8);
        }
    }

    std::vector<cv::Point2f> centers(minRect.size());

    for (size_t i = 0; i < minRect.size(); ++i)
    {
        minRect[i].points(rect_points);

        centers[i] = minRect[i].center;
        centroid.x += centers[i].x;
        centroid.y += centers[i].y;

        cv::circle(layout, centers[i], 4, BGR.green);

        for (int j = 0; j < 4; ++j)
            cv::line(layout, rect_points[j], rect_points[(j + 1) % 4], BGR.red, 5);
    }

    if (minRect.size() != 0)
    {
        centroid.x = centroid.x / (minRect.size());
        centroid.y = centroid.y / (minRect.size());
    }
    else
    {
        centroid.x = frame_.cols / 2;
        centroid.y = frame_.rows / 2;
    }

    cv::circle(layout, centroid, 16, BGR.blue, -1);

    pixel_dist_x_ = (frame_.cols / 2) - centroid.x;

    cv::imshow("mask", mask);
    cv::imshow("canny", canny);
    cv::imshow("erosion", erosion);
    cv::imshow("dilate", dilate);
    cv::imshow("opening", opening);
    cv::imshow("layout", layout);

    return true;
}

bool Cam_to_Wheel::Publish()
{
    dist_msg_.pixel_dist_x = pixel_dist_x_;

    pub_.publish(dist_msg_);

    return true;
}