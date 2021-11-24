#include <ros.h>
#include <object_tracking/wheel_msg.h>
#include <SparkFun_TB6612.h>

ros::NodeHandle nh;

void wheelCallback(const object_tracking::wheel_msg& wheel_msg)
{
    int right = wheel_msg.right;
    String str_right = String(right);
    nh.loginfo(str_right.c_str());
}

ros::Subscriber<object_tracking::wheel_msg> sub("wheel", &wheelCallback);

void setup()
{
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    nh.spinOnce();
    delay(100);//10 is too small that shitty nano cannot catch on
}