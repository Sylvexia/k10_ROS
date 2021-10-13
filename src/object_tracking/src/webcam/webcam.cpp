#include "webcam/webcam.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "webcam_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    WebCamera WebCamera(nh);

    while (ros::ok())
    {
        WebCamera.Capture();
        WebCamera.Publish();
        WebCamera.Imshow(640, 480);

        if (cv::waitKey(1) == 'q')
            break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}