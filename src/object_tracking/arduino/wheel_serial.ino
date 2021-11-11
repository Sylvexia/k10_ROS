#include <ros.h>
#include <object_tracking/wheel_msg.h>
#include <SparkFun_TB6612.h>

ros::NodeHandle nh;

void wheelCallback(object_tracking::wheel_msg &wheel_msg)
{
    digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN));
    Serial.println(wheel_msg.left);
}

ros::Subscriber<object_tracking::wheel_msg> sub("wheel", &wheelCallback);

void setup()
{
    Serial.begin(9600);
    Serial.print(123);
    pinMode(LED_BUILTIN, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    Serial.print(123);
    nh.spinOnce();
    delay(1);
}