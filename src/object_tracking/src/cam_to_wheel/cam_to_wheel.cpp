#include "cam_to_wheel/cam_to_wheel.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Cam_to_Wheel");
    ros::NodeHandle nh;

    Cam_to_Wheel cam_to_wheel(nh);

    cam_to_wheel.Recieve();

    ros::spin();

    return 0;
}