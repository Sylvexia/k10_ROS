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

bool Cam_to_Wheel::ImageProcess() //return the double dist_x_
{
    hsv_upper_bound_ = HSV.upper_green;
    hsv_lower_bound_ = HSV.lower_green;

    layout_ = cv::Mat::zeros(frame_.size(), CV_8SC3);

    pre_proc_ = ImagePreProc(frame_);
    //ImageWeightedCentroid(pre_proc_);
    ImageLargestContour(pre_proc_);

    cv::imshow("preproc", pre_proc_);
    cv::imshow("erosion", erosion_);
    cv::imshow("dilate", dilate_);
    cv::imshow("opening", opening_);
    cv::imshow("layout", layout_);

    return true;
}

bool Cam_to_Wheel::Publish()
{
    dist_msg_.pixel_dist_x = pixel_dist_x_;

    pub_.publish(dist_msg_);

    return true;
}

cv::Mat Cam_to_Wheel::ImagePreProc(cv::Mat input)
{
    cv::Point2d anchor = cv::Point2d(-1, -1);

    cv::GaussianBlur(input, blur_, cv::Size2d(11, 11), 0);
    cv::cvtColor(blur_, hsv_, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_, hsv_lower_bound_, hsv_upper_bound_, mask_);
    cv::erode(mask_, erosion_, std::vector<int>(3, 3), anchor, 6);
    cv::dilate(erosion_, dilate_, std::vector<int>(3, 3), anchor, 6);
    cv::morphologyEx(dilate_, opening_, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size2d(7, 7)), anchor, 1);
    cv::Canny(opening_, canny_, 40, 160);

    return opening_;
}

std::array<double, 2> Cam_to_Wheel::ImageWeightedCentroid(cv::Mat input)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    std::vector<cv::RotatedRect> minRect;
    std::array<double, 2> dist_point;
    cv::Point2f rect_points[4];
    cv::Point2f centroid(0.f, 0.f);

    cv::findContours(input, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    minRect.reserve(contours.size());

    for (size_t i = 0; i < contours.size(); ++i) //filtering small contour and push into minRect array
    {
        if (cv::contourArea(contours[i]) > 500)
        {
            minRect.emplace_back(cv::minAreaRect(contours[i]));
            cv::drawContours(layout_, contours, i, BGR.white, -1, cv::LINE_8);
        }
    }

    std::vector<cv::Point2f> centers(minRect.size());

    int total_weight_x = 0;
    int total_weight_y = 0;

    for (size_t i = 0; i < minRect.size(); ++i)
    {
        minRect[i].points(rect_points);

        cv::Rect weight = minRect[i].boundingRect();

        centers[i] = minRect[i].center;
        centroid.x += centers[i].x * weight.x;
        centroid.y += centers[i].y * weight.y;
        total_weight_x += weight.x;
        total_weight_y += weight.y;

        cv::circle(layout_, centers[i], 4, BGR.green);

        for (int j = 0; j < 4; ++j)
            cv::line(layout_, rect_points[j], rect_points[(j + 1) % 4], BGR.red, 5);
    }

    if (minRect.size() != 0)
    {
        centroid.x = centroid.x / total_weight_x;
        centroid.y = centroid.y / total_weight_y;
    }
    else
    {
        centroid.x = input.cols / 2;
        centroid.y = input.rows / 2;
    }

    cv::circle(layout_, centroid, 16, BGR.blue, -1);

    dist_point[0] = (input.cols / 2) - centroid.x;
    dist_point[1] = (input.rows / 2) - centroid.y;

    pixel_dist_x_ = dist_point[0];
    pixel_dist_y_ = dist_point[1];

    return dist_point;
}

std::array<double, 2> Cam_to_Wheel::ImageLargestContour(cv::Mat input)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    std::vector<double> contours_areas;
    std::array<double, 2> dist_point;

    cv::findContours(input, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (contours.size() == 0)   //early return, preventing null number
    {
        dist_point[0] = (input.cols / 2);
        dist_point[1] = (input.rows / 2);
        pixel_dist_x_ = 0;
        pixel_dist_y_ = 0;
        return dist_point;
    }

    contours_areas.reserve(contours.size());

    for (size_t i = 0; i < contours.size(); ++i)
        contours_areas.emplace_back(cv::contourArea(contours[i]));

    auto max_contour_area = std::max_element(contours_areas.begin(), contours_areas.end());
    int max_index = std::distance(contours_areas.begin(), max_contour_area);

    cv::RotatedRect minRect = cv::minAreaRect(contours[max_index]);

    dist_point[0] = (input.cols / 2) - minRect.center.x;
    dist_point[1] = (input.rows / 2) - minRect.center.y;

    cv::drawContours(layout_, contours, max_index, BGR.white, -1, cv::LINE_8);
    cv::circle(layout_, minRect.center, 16, BGR.blue, -1);

    pixel_dist_x_ = dist_point[0];
    pixel_dist_y_ = dist_point[1];

    return dist_point;
}
