#include "Drivebase.h"
#include "RobotMap.h"
#include <Arduino.h>
#include <Motors.h>
#include <Encoders.h>

namespace DriveBase {
    // Tank Drive Motors
    Encoder encoderLeft(RobotMap::ENC_LEFT_A, RobotMap::ENC_LEFT_B);
    Encoder encoderRight(RobotMap::ENC_RIGHT_A, RobotMap::ENC_RIGHT_B);
    Motor motorLeft(RobotMap::MOTOR_LEFT_A, RobotMap::MOTOR_LEFT_B, &encoderLeft);
    Motor motorRight(RobotMap::MOTOR_RIGHT_A, RobotMap::MOTOR_RIGHT_B, &encoderRight);

    // Pivot Motors
    Encoder encoderPivotFront(RobotMap::ENC_PIVOT_FRONT_A, RobotMap::ENC_PIVOT_FRONT_B);
    Encoder encoderPivotRear(RobotMap::ENC_PIVOT_REAR_A, RobotMap::ENC_PIVOT_REAR_B);
    Motor motorPivotFront(RobotMap::MOTOR_PIVOT_FRONT_A, RobotMap::MOTOR_PIVOT_FRONT_B, &encoderPivotFront);
    Motor motorPivotRear(RobotMap::MOTOR_PIVOT_REAR_A, RobotMap::MOTOR_PIVOT_REAR_B, &encoderPivotRear);

    int predictiveHeading = 0;

    void begin() {
        // Initialize motor PWM driver
        Motor::initializePWM();
        
        // Initialize all encoders
        encoderLeft.begin();
        encoderRight.begin();
        encoderPivotFront.begin();
        encoderPivotRear.begin();

        // Attach interrupts for all encoders
        attachInterrupt(digitalPinToInterrupt(encoderLeft.getPinA()), []()
                        { encoderLeft.updateCount(); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderRight.getPinA()), []()
                        { encoderRight.updateCount(); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPivotFront.getPinA()), []()
                        { encoderPivotFront.updateCount(); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPivotRear.getPinA()), []()
                        { encoderPivotRear.updateCount(); }, CHANGE);

        // Stop all motors initially
        motorLeft.coast();
        motorRight.coast();
        motorPivotFront.coast();
        motorPivotRear.coast();
    }

    void update() {
        // Update control for all motors
        motorLeft.updateControl();
        motorRight.updateControl();
        motorPivotFront.updateControl();
        motorPivotRear.updateControl();
    }
}