#include "Drivebase.h"
#include "RobotMap.h"
#include <Arduino.h>
#include <Motors.h>
#include <Encoders.h>

namespace DriveBase {
    // Define the global variables here
    Encoder encoderFL(RobotMap::ENC_FLA, RobotMap::ENC_FLB);
    Encoder encoderFR(RobotMap::ENC_FRA, RobotMap::ENC_FRB);
    Encoder encoderBL(RobotMap::ENC_BLA, RobotMap::ENC_BLB);
    Encoder encoderBR(RobotMap::ENC_BRA, RobotMap::ENC_BRB);

    Motor motorFL(RobotMap::MOTOR_FLA, RobotMap::MOTOR_FLB, &encoderFL);
    Motor motorFR(RobotMap::MOTOR_FRA, RobotMap::MOTOR_FRB, &encoderFR);
    Motor motorBL(RobotMap::MOTOR_BLA, RobotMap::MOTOR_BLB, &encoderBL);
    Motor motorBR(RobotMap::MOTOR_BRA, RobotMap::MOTOR_BRB, &encoderBR);

    int predictiveHeading = 0;

    void begin() {
        // Initialize motor PWM driver
        Motor::initializePWM();
        // Initialize all encoders
        encoderFL.begin();
        encoderFR.begin();
        encoderBL.begin();
        encoderBR.begin();

        // Attach interrupts for all encoders
        attachInterrupt(digitalPinToInterrupt(encoderFL.getPinA()), []()
                        { encoderFL.updateCount(); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderFR.getPinA()), []()
                        { encoderFR.updateCount(); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderBL.getPinA()), []()
                        { encoderBL.updateCount(); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderBR.getPinA()), []()
                        { encoderBR.updateCount(); }, CHANGE);



        // Stop all motors initially
        motorFL.coast();
        motorFR.coast();
        motorBL.coast();
        motorBR.coast();
    }

    void update() {
        // Update control for all motors
        motorFL.updateControl();
        motorFR.updateControl();
        motorBL.updateControl();
        motorBR.updateControl();
    }
}