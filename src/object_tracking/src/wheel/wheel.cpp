#include "wheel/wheel.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel");
    ros::NodeHandle nh;

    double kp, ki, kd;

    nh.getParam("/cam_to_wheel_node/kp", kp);
    nh.getParam("/cam_to_wheel_node/ki", ki);
    nh.getParam("/cam_to_wheel_node/kd", kd);

    PID pid(kp, ki, kd, 0.0);

    Wheel wheel(nh, pid);

    wheel.Run();

    ros::spin();

    return 0;
}