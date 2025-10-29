#include "drivebase/Drivebase.h"
#include "RobotMap.h"
#include <Arduino.h>
#include <Motors.h>
#include <Encoders.h>

    
namespace DriveBase {
    float targetxvel = 0.0f;
    float targetyvel = 0.0f;
    float targetHeading = 0.0f;
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

        if(driveMode == HEADING_CONTROL) {
            // Turn on the spot using heading PID
            double currentHeading = imu.getHeading();
            headingError = headingSetpoint - currentHeading;

            // Normalize error to [-180, 180]
            while (headingError > 180.0) headingError -= 360.0;
            while (headingError < -180.0) headingError += 360.0;

            unsigned long now = millis();
            double dt = (now - headingLastTime) / 1000.0; // seconds

            headingIntegral += headingError * dt;
            double headingDerivative = (headingError - headingLastError) / dt;

            double output = headingKp * headingError + headingKi * headingIntegral + headingKd * headingDerivative;

            // Clamp output to motor speed limits
            output = constrain(output, -4095, 4095);

            // Turn on the spot: left and right motors in opposite directions
            motorLeft.setSpeed(output);
            motorRight.setSpeed(-output);

            headingLastError = headingError;
            headingLastTime = now;
            return;
        }

        if (driveMode == STRAIGHT_LINE_CONTROL) {
            // Both motors start at high speed (4096)
            // As the robot veers away from targetHeading, reduce speed of one side to correct
            float headingError = DriveBase::imu.getHeading() - targetHeading;
            float correction = headingError / 180.0f * 4095 * 15; // scale correction

            DriveBase::motorLeft.setSpeed(4096 + correction);  // Reduce left if error positive
            DriveBase::motorRight.setSpeed(4096 - correction); // Reduce right if error negative

            // Calculate angle between target x and y velocity
            float angle = atan2(targetyvel, targetxvel) * 180.0f / PI;
            DriveBase::motorPivotLeft.setTargetPosition(angle);
            DriveBase::motorPivotRight.setTargetPosition(angle);
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
        // Configure position PID gains for pivot motors - conservative tuning to reduce oscillation
        motorPivotRight.setPositionPID(3.0, 0.0, 0.0);  // Conservative gains for right motor (higher resistance)
        motorPivotRight.setPositionTolerance(100);         // Relaxed tolerance for reliable stopping

        motorPivotLeft.setPositionPID(2.0, 0.0, 0.0);  // Conservative gains for left motor (more power)
        motorPivotLeft.setPositionTolerance(100);          // Relaxed tolerance for reliable stopping
    }

}