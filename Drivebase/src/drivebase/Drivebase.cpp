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
    Encoder encoderPivotRight(RobotMap::ENC_PIVOT_FRONT_A, RobotMap::ENC_PIVOT_FRONT_B);
    Encoder encoderPivotLeft(RobotMap::ENC_PIVOT_REAR_A, RobotMap::ENC_PIVOT_REAR_B);
    Motor motorPivotRight(RobotMap::MOTOR_PIVOT_FRONT_A, RobotMap::MOTOR_PIVOT_FRONT_B, &encoderPivotRight);
    Motor motorPivotLeft(RobotMap::MOTOR_PIVOT_REAR_A, RobotMap::MOTOR_PIVOT_REAR_B, &encoderPivotLeft);

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
        encoderPivotRight.begin();
        encoderPivotLeft.begin();

        encoderPivotRight.setCPR(5900);
        encoderPivotLeft.setCPR(5738);

        attachInterrupt(digitalPinToInterrupt(encoderLeft.getPinA()), []()
                        { encoderLeft.updateCount(); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderRight.getPinA()), []()
                        { encoderRight.updateCount(); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPivotRight.getPinA()), []()
                        { encoderPivotRight.updateCount(); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPivotLeft.getPinA()), []()
                        { encoderPivotLeft.updateCount(); }, CHANGE);

        // Stop all motors initially
        motorLeft.coast();
        motorRight.coast();
        motorPivotRight.coast();
        motorPivotLeft.coast();

        encoderLeft.setInverted(true);
        encoderRight.setInverted(false);

        encoderPivotRight.setInverted(true);   // Right encoder inverted
        encoderPivotLeft.setInverted(true);    // Left encoder inverted for consistency
        motorPivotLeft.setInverted(true);      // Left motor inverted - negative PWM drives reverse
        motorPivotRight.setInverted(true);     // Right motor inverted - negative PWM drives reverse

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