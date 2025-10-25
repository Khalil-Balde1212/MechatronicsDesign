#ifndef DRIVEBASE_H
#define DRIVEBASE_H
#include "Motors.h"
#include "Encoders.h"
#include "RobotMap.h"
#include <IMU.h>

namespace DriveBase {
    // Tank Drive Motors - Left and Right for forward/backward motion
    extern Encoder encoderLeft;
    extern Encoder encoderRight;
    extern Motor motorLeft;
    extern Motor motorRight;

    // Pivot System - Single pivot controlled by two motors
    extern Encoder encoderPivotFront;
    extern Encoder encoderPivotRear;
    extern Motor motorPivotFront;
    extern Motor motorPivotRear;

    extern IMUController imu;

    // Target speeds for heading-corrected driving
    extern double targetLeftSpeed;
    extern double targetRightSpeed;

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
}
#endif