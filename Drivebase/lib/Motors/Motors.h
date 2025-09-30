#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

namespace Motors {
    const int MOTOR_FLA = 14;
    const int MOTOR_FLB = 15;
    const int MOTOR_FRA = 5;
    const int MOTOR_FRB = 4;
    const int MOTOR_BLA = 13;
    const int MOTOR_BLB = 12;
    const int MOTOR_BRA = 7;
    const int MOTOR_BRB = 6;
    
    extern Adafruit_PWMServoDriver pwm;
    
    void initialize();
    
    void setSpeed(int motorPin1, int motorPin2, int speed);
    void setSpeeds(float left, float right);
    void stopAll();
    
    void setSpeedFL(int speed);
    void setSpeedFR(int speed);
    void setSpeedBL(int speed);
    void setSpeedBR(int speed);
}

#endif