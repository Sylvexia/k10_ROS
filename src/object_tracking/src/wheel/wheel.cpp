#include "wheel/wheel.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel");
    ros::NodeHandle nh;

    Wheel wheel(nh);

    wheel.Run();

    ros::spin();

    return 0;
}