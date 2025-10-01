#include "Motors.h"

namespace Motors {
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
    
    void initialize() {
        pwm.begin();
        pwm.setPWMFreq(400);
        stopAll();
    }
    
    void setSpeed(int motorPin1, int motorPin2, int speed) {
        speed = constrain(speed, -4095, 4095);

        if (speed != 0 && abs(speed) < 1000) {
            speed = (speed > 0) ? 1000 : -1000;
        }
        
        if (speed > 0) {
            pwm.setPWM(motorPin1, 0, speed);
            pwm.setPWM(motorPin2, 0, 0);
        } else if (speed < 0) {
            pwm.setPWM(motorPin1, 0, 0);
            pwm.setPWM(motorPin2, 0, -speed);
        } else {
            pwm.setPWM(motorPin1, 0, 0);
            pwm.setPWM(motorPin2, 0, 0);
        }
    }
    
    void setSpeeds(float left, float right) {
        left = left * 4096;
        right = right * 4096;

        setSpeed(MOTOR_FLA, MOTOR_FLB, left);
        setSpeed(MOTOR_BLA, MOTOR_BLB, left);
        setSpeed(MOTOR_FRA, MOTOR_FRB, right);
        setSpeed(MOTOR_BRA, MOTOR_BRB, right);
    }
    
    void stopAll() {
        setSpeeds(0, 0);
    }
    
    void setSpeedFL(int speed) {
        setSpeed(MOTOR_FLA, MOTOR_FLB, speed);
    }
    
    void setSpeedFR(int speed) {
        setSpeed(MOTOR_FRA, MOTOR_FRB, speed);
    }
    
    void setSpeedBL(int speed) {
        setSpeed(MOTOR_BLA, MOTOR_BLB, speed);
    }
    
    void setSpeedBR(int speed) {
        setSpeed(MOTOR_BRA, MOTOR_BRB, speed);
    }
}
