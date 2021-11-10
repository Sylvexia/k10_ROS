#include "cam_to_wheel/cam_to_wheel.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Cam_to_Wheel");
    ros::NodeHandle nh;

    double kp, ki, kd;

    nh.getParam("/cam_to_wheel_node/kp", kp);
    nh.getParam("/cam_to_wheel_node/ki", ki);
    nh.getParam("/cam_to_wheel_node/kd", kd);

    PID pid(kp, ki, kd, 0.0);

    Cam_to_Wheel cam_to_wheel(nh, pid);

    cam_to_wheel.Recieve();

    ros::spin();

    return 0;
}