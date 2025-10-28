#include <Arduino.h>
#include "drivebase/Drivebase.h"

namespace DriveBase
{
    // Heading PID variables
    extern double headingKp, headingKi, headingKd;
    extern double headingSetpoint;
    extern double headingError, headingLastError, headingIntegral;

    extern unsigned long headingLastTime;
    extern bool headingPIDEnabled;

    extern IMUController imu;
    // Heading PID functions
    void setHeadingPID(double kp, double ki, double kd)
    {
        headingKp = kp;
        headingKi = ki;
        headingKd = kd;
    }

    void setTargetHeading(double heading)
    {
        headingSetpoint = heading;
        resetHeadingPID();
    }

    void enableHeadingPID(bool enable)
    {
        headingPIDEnabled = enable;
        if (enable)
        {
            resetHeadingPID();
        }
    }

    void resetHeadingPID()
    {
        headingError = 0.0;
        headingLastError = 0.0;
        headingIntegral = 0.0;
        headingLastTime = millis();
    }

    double calculateHeadingCorrection()
    {
        unsigned long currentTime = millis();
        double deltaTime = (currentTime - headingLastTime) / 1000.0;

        double currentHeading = imu.getYaw();
        headingError = headingSetpoint - currentHeading;

        // Wrap error to [-180, 180] degrees
        while (headingError > 180.0)
            headingError -= 360.0;
        while (headingError < -180.0)
            headingError += 360.0;

        // Calculate integral term
        headingIntegral += headingError * deltaTime;
        double derivative = (headingError - headingLastError) / deltaTime;

        double output = headingKp * headingError + headingKi * headingIntegral + headingKd * derivative;

        // // Clamp output to reasonable range
        // output = constrain(output, -0.5, 0.5);

        // Update for next iteration
        headingLastError = headingError;
        headingLastTime = currentTime;

        return output;
    }
}