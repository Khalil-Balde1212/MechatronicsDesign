#ifndef DRIVEBASE_H
#define DRIVEBASE_H
#include "Motors.h"
#include "Encoders.h"
#include "RobotMap.h"

namespace DriveBase {
    extern Encoder encoderFL;
    extern Encoder encoderFR;
    extern Encoder encoderBL;
    extern Encoder encoderBR;

    extern Motor motorFL;
    extern Motor motorFR;
    extern Motor motorBL;
    extern Motor motorBR;

    void begin();
    void update();

    extern int predictiveHeading;
}
#endif