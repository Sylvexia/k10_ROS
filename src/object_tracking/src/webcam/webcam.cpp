#include "webcam/webcam.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "webcam");
    ros::NodeHandle nh;
    ros::Rate r(10);

    WebCamera WebCamera(nh);

    cv::VideoCapture cap(0);
    
    cv::Mat frame;
    cv::Mat resized_frame;

    if (!cap.isOpened())
        return -1;
    
    cv::namedWindow("Window_Default", cv::WINDOW_NORMAL);

    while(true)
    {
        ros::spinOnce();
        cap >> frame;
        cv::resize(frame,resized_frame,cv::Size(600,400),cv::INTER_LINEAR);
        cv::imshow("resized_frame", resized_frame);
        //r.sleep();
		if(cv::waitKey(1)=='q')
			break;
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}