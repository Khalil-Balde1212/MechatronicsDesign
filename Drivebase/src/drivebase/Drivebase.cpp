#include "drivebase/Drivebase.h"
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
    Encoder encoderPivotRight(RobotMap::ENC_PIVOT_RIGHT_A, RobotMap::ENC_PIVOT_RIGHT_B);
    Encoder encoderPivotLeft(RobotMap::ENC_PIVOT_LEFT_A, RobotMap::ENC_PIVOT_LEFT_B);
    Motor motorPivotRight(RobotMap::MOTOR_PIVOT_RIGHT_A, RobotMap::MOTOR_PIVOT_RIGHT_B, &encoderPivotRight);
    Motor motorPivotLeft(RobotMap::MOTOR_PIVOT_LEFT_A, RobotMap::MOTOR_PIVOT_LEFT_B, &encoderPivotLeft);

    //Gyro
    IMUController imu;

    // Heading PID variables
    double headingKp = 0.5, headingKi = 0.0, headingKd = 0.1;
    double headingSetpoint = 0.0;
    double headingError = 0.0, headingLastError = 0.0, headingIntegral = 0.0;
    unsigned long headingLastTime = 0;
    bool headingPIDEnabled = false;

    // Target speeds
    double targetLeftSpeed = 0.0;
    double targetRightSpeed = 0.0;

    int predictiveHeading = 0;

    void begin() {
        // Initialize all encoders
        encoderLeft.begin();
        encoderRight.begin();
        encoderPivotLeft.begin();
        encoderPivotRight.begin();

        encoderLeft.setCPR(RobotMap::ENC_LEFT_CPR);
        encoderRight.setCPR(RobotMap::ENC_RIGHT_CPR);
        encoderPivotLeft.setCPR(RobotMap::ENC_PIVOT_LEFT_CPR);
        encoderPivotRight.setCPR(RobotMap::ENC_PIVOT_RIGHT_CPR);

        encoderLeft.setInverted(RobotMap::ENC_LEFT_INVERTED);
        encoderRight.setInverted(RobotMap::ENC_RIGHT_INVERTED);
        encoderPivotLeft.setInverted(RobotMap::ENC_PIVOT_LEFT_INVERTED);
        encoderPivotRight.setInverted(RobotMap::ENC_PIVOT_RIGHT_INVERTED);

        attachInterrupt(digitalPinToInterrupt(encoderLeft.getPinA()), []()
                        { encoderLeft.updateCount(); }, RISING);
        attachInterrupt(digitalPinToInterrupt(encoderRight.getPinA()), []()
                        { encoderRight.updateCount(); }, RISING);
        attachInterrupt(digitalPinToInterrupt(encoderPivotLeft.getPinA()), []()
                        { encoderPivotLeft.updateCount(); }, RISING);
        attachInterrupt(digitalPinToInterrupt(encoderPivotRight.getPinA()), []()
                        { encoderPivotRight.updateCount(); }, RISING);

        // Stop all motors initially
        motorLeft.coast();
        motorRight.coast();
        motorPivotRight.coast();
        motorPivotLeft.coast();


        // Initialize PWM driver (only needs to be done once for all motors)
        motorLeft.initializePWM();

        // Initialize IMU
        imu.begin();

        // Reset encoders to 0 (assuming motors start at 0 degree position)
        resetEncoders();

        // Initialize heading PID
        headingLastTime = millis();

        configurePIDs();
    }

    void update() {
        // Update control for all motors
        motorLeft.updateControl();
        motorRight.updateControl();
        motorPivotRight.updateControl();
        motorPivotLeft.updateControl();
        imu.update();

        // Apply heading correction if enabled
        if (headingPIDEnabled) {
            double correction = calculateHeadingCorrection();
            // Apply correction to tank drive
            motorLeft.setSpeed(targetLeftSpeed + correction);
            motorRight.setSpeed(targetRightSpeed - correction);
        }
    }

    void resetEncoders() {
        encoderLeft.reset();
        encoderRight.reset();
        encoderPivotRight.reset();
        encoderPivotLeft.reset();
    }




    void configurePIDs() {
        // Configure position PID gains for pivot motors - conservative tuning to reduce oscillation
        motorPivotRight.setPositionPID(3.0, 0.0, 0.0);  // Conservative gains for right motor (higher resistance)
        motorPivotRight.setPositionTolerance(100);         // Relaxed tolerance for reliable stopping

        motorPivotLeft.setPositionPID(2.0, 0.0, 0.0);  // Conservative gains for left motor (more power)
        motorPivotLeft.setPositionTolerance(100);          // Relaxed tolerance for reliable stopping
    }

}