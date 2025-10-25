#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Encoders.h"
#include "IMU.h"

#include "CommandInterpretter.h"
#include "RobotMap.h"
#include <Motors.h>

#include "drivebase/Drivebase.h"

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    CommandInterpreter::begin();
    DriveBase::begin();

    // Disable the built-in heading PID to use motor's position control
    DriveBase::enableHeadingPID(false);
    

    CommandInterpreter::registerCommand({"targetPivPos", [](const std::string* args)
        {
            long targetPos = std::stol(*args);
            DriveBase::motorPivotFront.setTargetPosition(targetPos * DriveBase::motorPivotFront.getEncoder()->getCPR() / 360); // Convert degrees to ticks
            DriveBase::motorPivotRear.setTargetPosition(targetPos * DriveBase::motorPivotRear.getEncoder()->getCPR() / 360); // Convert degrees to ticks
            DriveBase::motorPivotFront.enableRawPositionControl(true);
            DriveBase::motorPivotRear.enableRawPositionControl(true);
            Serial.print("Set target position to: ");
            Serial.println(targetPos);
        },
        "Usage: targetPivPos <position_ticks> \n Sets the target position for pivot motors in degrees."
        });

    
    DriveBase::motorPivotFront.enableRawPositionControl(true);
}

static unsigned long lastPrintTime = 0;

void loop()
{
    // Update motor control (this runs the PID for ALL motors)
    CommandInterpreter::periodic();
    DriveBase::update();  // This calls updateControl() for all motors
    
    
    unsigned long currentTime = millis();
    // Print status every 500ms
    if (currentTime - lastPrintTime >= 500)
    {
        long currentPos = DriveBase::motorPivotFront.getCurrentPosition();
        long targetPos = DriveBase::motorPivotFront.getTargetPosition();    
        long error = targetPos - currentPos;
        
        Serial.print("Target: ");
        Serial.print(targetPos);
        Serial.print(" | Current: ");
        Serial.print(currentPos);
        Serial.print(" | Error: ");
        Serial.print(error);
        Serial.print(" | IMU Yaw: ");
        Serial.print(DriveBase::imu.getYaw(), 1);
        Serial.print("° | At Target: ");
        Serial.print(DriveBase::motorPivotFront.isPositionAtTarget() ? "YES" : "NO");
        Serial.print(" | Mode: ");
        Serial.println(DriveBase::motorPivotFront.getControlMode());
        
        lastPrintTime = currentTime;
    }
}
