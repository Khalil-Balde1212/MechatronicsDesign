#include "Motors.h"
#include "Encoders.h"
#include "RobotMap.h"
#include <IMU.h>

#ifndef DRIVEBASE_H
#define DRIVEBASE_H


namespace DriveBase {
    // Tank Drive Motors - Left and Right for forward/backward motion
    extern Encoder encoderLeft;
    extern Encoder encoderRight;
    extern Motor motorLeft;
    extern Motor motorRight;

    // Pivot System - Single pivot controlled by two motors
    extern Encoder encoderPivotRight;
    extern Encoder encoderPivotLeft;
    extern Motor motorPivotRight;
    extern Motor motorPivotLeft;

    extern IMUController imu;

    // Target speeds for heading-corrected driving
    extern double targetLeftSpeed;
    extern double targetRightSpeed;
    extern double pivotAngle;


        // trajetory driving variables
    extern float targetVx;
    extern float targetVy;
    extern float targetYawRate;

    enum DriveMode {
        RAW_CONTROL, 
        HEADING_CONTROL,
        TRAJECTORY_CONTROL
    };

    static DriveMode driveMode;

    void begin();
    void update();
    
    // Heading PID functions
    void setHeadingPID(double kp, double ki, double kd);
    void setTargetHeading(double heading);
    void enableHeadingPID(bool enable);
    void resetHeadingPID();
    double calculateHeadingCorrection();

    // Position PIDs
    void configurePIDs();
    void resetEncoders();

    void calculateTrajectory(float vx, float vy, float yawRate);

    // Trajectory control functions
    void setTargetYawRate(float yawRate);
    void resetHeadingMaintenance();


    // raw controls
    void setRawSpeedLeft(double speed);
    void setRawSpeedRight(double speed);
    void setRawSpeedLeftPivot(double speed);
    void setRawSpeedRightPivot(double speed);

    // position controls
    void setPositionLeft(long positionTicks);
    void setPositionRight(long positionTicks);
    void setPositionLeftPivot(long positionTicks);
    void setPositionRightPivot(long positionTicks);
}

#endif