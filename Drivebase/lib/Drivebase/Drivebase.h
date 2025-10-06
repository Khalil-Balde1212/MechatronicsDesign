#ifndef DRIVEBASE_H
#define DRIVEBASE_H
#include <Motors.h>
#include <Encoders.h>

namespace DriveBase {
    Motor motorFL, motorFR, motorBL, motorBR;
    Encoder encoderFL, encoderFR, encoderBL, encoderBR;

    void begin();
    void update();


}
#endif