#include "webcam/webcam.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "webcam");
    ros::NodeHandle nh;
    ros::Rate r(10);

    //WebCamera WebCamera(nh);

    cv::VideoCapture cap(0);
    
    cv::Mat frame;
    cv::Mat gray;

    if (!cap.isOpened())
        return -1;

    
    cv::namedWindow("Window_Default", cv::WINDOW_NORMAL);

    while(ros::ok())
    {
        ros::spinOnce();
        printf("the fuck");
        cap >> frame;
        cv::imshow("frame", frame);
        r.sleep();
		if(cv::waitKey(1)=='q')
			break;

    }
    return 0;
}