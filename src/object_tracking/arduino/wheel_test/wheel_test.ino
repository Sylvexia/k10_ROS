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

Motor rightMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor leftMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup()
{
    ;
}

void loop()
{
    rightMotor.drive(255, 1000);
    rightMotor.drive(-255, 1000);
    rightMotor.brake();
    delay(1000);

    leftMotor.drive(255, 1000);
    leftMotor.drive(-255, 1000);
    leftMotor.brake();
    delay(1000);

    forward(rightMotor, leftMotor, 150);
    delay(1000);

    back(rightMotor, leftMotor, 150);
    delay(1000);

    brake(rightMotor, leftMotor);
    delay(1000);

    left(leftMotor, rightMotor, 150);
    delay(1000);

    right(leftMotor, rightMotor, 150);
    delay(1000);

    brake(leftMotor, rightMotor);
    delay(1000);
}
