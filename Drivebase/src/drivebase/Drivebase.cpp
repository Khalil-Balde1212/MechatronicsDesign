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

    // Trajectory driving variables
    float targetVx = 0.0f;
    float targetVy = 0.0f;
    float targetYawRate = 0.0f;

    // Heading maintenance state
    static double lastMaintainedHeading = 0.0;
    static bool headingInitialized = false;

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
            return; 
        }



        if (driveMode == TRAJECTORY_CONTROL) {
            // Enable speed control for tank drive motors
            motorLeft.enableSpeedControl(true);
            motorRight.enableSpeedControl(true);
            
            float pivotAngle = atan2(targetVy, targetVx) * 180.0 / PI; // in degrees

            motorPivotLeft.setTargetPosition(pivotAngle);
            motorPivotRight.setTargetPosition(pivotAngle);

            float magnitude = sqrt(targetVx * targetVx + targetVy * targetVy)/(RobotMap::WHEEL_RADIUS_MM*2*PI/1000.0); // pythagoras my boi

            // Calculate yaw correction from multiple sources
            double yawCorrection = 0.0;
            
            // Choose control mode based on targetYawRate
            if (abs(targetYawRate) < 0.01) { 
                // Maintain current heading when omega ≈ 0
                if (!headingInitialized) {
                    lastMaintainedHeading = imu.getYaw();
                    setTargetHeading(lastMaintainedHeading);
                    enableHeadingPID(true);
                    headingInitialized = true;
                }
                yawCorrection = calculateHeadingCorrection();
            } else { 
                // Active yaw rate control when omega ≠ 0
                enableHeadingPID(false);
                headingInitialized = false;
                
                // Use yaw rate PID for turning
                static double yawKp = 0.4, yawKi = 0.0, yawKd = 0.08;
                static double yawError = 0.0, yawLastError = 0.0, yawIntegral = 0.0;
                static unsigned long yawLastTime = 0;

                if (yawLastTime == 0) yawLastTime = millis();
                double currentYawRate = imu.getAngularVelocityZRad();
                yawError = targetYawRate - currentYawRate;
                unsigned long now = millis();
                double dt = (now - yawLastTime) / 1000.0;
                yawIntegral += yawError * dt;
                double yawDerivative = (yawError - yawLastError) / dt;
                yawCorrection = yawKp * yawError + yawKi * yawIntegral + yawKd * yawDerivative;
                yawLastError = yawError;
                yawLastTime = now;
            }

            // Apply yaw correction to tank drive
            motorLeft.setTargetSpeed(magnitude + yawCorrection);
            motorRight.setTargetSpeed(magnitude - yawCorrection);

            return;
        }
    }

    void resetEncoders() {
        encoderLeft.reset();
        encoderRight.reset();
        encoderPivotRight.reset();
        encoderPivotLeft.reset();
    }




    void configurePIDs() {
        // Configure speed PID gains for tank drive motors
        motorLeft.setSpeedPID(2.0, 0.1, 0.0);      // Speed control for left motor
        motorLeft.setSpeedTolerance(0.1);            // 0.1 RPS tolerance
        
        motorRight.setSpeedPID(2.0, 0.1, 0.0);     // Speed control for right motor  
        motorRight.setSpeedTolerance(0.1);           // 0.1 RPS tolerance

        // Configure position PID gains for pivot motors - conservative tuning to reduce oscillation
        motorPivotRight.setPositionPID(3.0, 0.0, 0.0);  // Conservative gains for right motor (higher resistance)
        motorPivotRight.setPositionTolerance(100);         // Relaxed tolerance for reliable stopping

        motorPivotLeft.setPositionPID(2.0, 0.0, 0.0);  // Conservative gains for left motor (more power)
        motorPivotLeft.setPositionTolerance(100);          // Relaxed tolerance for reliable stopping
    }

    void calculateTrajectory(float vx, float vy, float yawRate) {
        targetVx = vx;
        targetVy = vy;
        targetYawRate = yawRate;
        driveMode = TRAJECTORY_CONTROL;
    }

    void setTargetYawRate(float yawRate) {
        targetYawRate = yawRate;
    }

    void resetHeadingMaintenance() {
        // Reset heading maintenance state for next trajectory
        headingInitialized = false;
        enableHeadingPID(false);
    }

}