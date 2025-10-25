#ifndef DRIVEBASE_H
#define DRIVEBASE_H
#include "Motors.h"
#include "Encoders.h"
#include "RobotMap.h"

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

    void begin();
    void update();

    extern int predictiveHeading;
}
#endif