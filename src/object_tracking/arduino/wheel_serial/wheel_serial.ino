#include <ros.h>
#include <object_tracking/wheel_msg.h>
#include <SparkFun_TB6612.h>

#define PWMA 5
#define AIN2 6
#define AIN1 7
#define STBY 8
#define BIN1 9
#define BIN2 10
#define PWMB 11
#define GND 12

const int offsetA = 1;
const int offsetB = 1;

int right_num = 0;
int left_num = 0;

char right_char[8];
char left_char[8];

char right_log[16];
char left_log[16];

uint16_t delay_t = 100;

Motor rightMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor leftMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

ros::NodeHandle nh;

void wheelCallback(const object_tracking::wheel_msg &wheel_msg)
{
    right_num = wheel_msg.right;
    itoa(right_num, right_char, 10);
    strcpy(right_log, "R:");
    strcat(right_log, right_char);
    nh.loginfo(right_log);
    rightMotor.drive(right_num);

    left_num = wheel_msg.left;
    itoa(left_num, left_char, 10);
    strcpy(left_log, "L:");
    strcat(left_log, left_char);
    nh.loginfo(left_log);
    leftMotor.drive(left_num);
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
    delay(100); //10 is too small that shitty nano cannot catch on
}