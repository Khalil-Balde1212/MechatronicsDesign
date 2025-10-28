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
    



        CommandInterpreter::registerCommand({"rawRightSpeed", [](const std::string* args)
            {
                Serial.print("Setting raw speed for pivot motor to: ");
                Serial.println(args->c_str());
                double speed = std::stod(args[0]);
                DriveBase::setRawSpeedRight(speed);
            },
        "Usage: rawRightSpeed \n Enables raw position control for pivot motors."
        });

        CommandInterpreter::registerCommand({"rawLeftSpeed", [](const std::string* args)
        {
            Serial.print("Setting raw left speed for pivot motor to: ");
            Serial.println(args->c_str());
            double speed = std::stod(args[0]);
            DriveBase::setRawSpeedLeft(speed);
        },
        "Usage: rawLeftSpeed \n Enables raw position control for pivot motors."
        });

        CommandInterpreter::registerCommand({"rawRightPivotSpeed", [](const std::string* args)
        {
            Serial.print("Setting raw speed for pivot motor to: ");
            Serial.println(args->c_str());
            double speed = std::stod(args[0]);
            DriveBase::setRawSpeedRightPivot(speed);
        },
        "Usage: rawRightPivotSpeed \n Enables raw position control for pivot motors."
        });

        CommandInterpreter::registerCommand({"rawLeftPivotSpeed", [](const std::string* args)
        {
            Serial.print("Setting raw left speed for pivot motor to: ");
            Serial.println(args->c_str());
            double speed = std::stod(args[0]);
            DriveBase::setRawSpeedLeftPivot(speed);
        },
        "Usage: rawLeftPivotSpeed \n Enables raw position control for pivot motors."
        });

        CommandInterpreter::registerCommand({"rawPivotSpeed", [](const std::string* args)
        {
            Serial.print("Setting raw speed for pivot motor to: ");
            Serial.println(args->c_str());
            double speed = std::stod(args[0]);
            DriveBase::setRawSpeedRightPivot(speed);
            DriveBase::setRawSpeedLeftPivot(speed);
        },
        "Usage: rawPivotSpeed \n Enables raw position control for pivot motors."
        });


        CommandInterpreter::registerCommand({"lp", [](const std::string* args)
        {
            DriveBase::setPositionLeft(std::stoi(args[0]));
            Serial.println("Left position set.");
        },
        "Usage: lp <position_ticks> \n Resets encoders and sets left wheel to target position in ticks."
        });

        CommandInterpreter::registerCommand({"rp", [](const std::string* args)
        {
            DriveBase::setPositionRight(std::stoi(args[0]));
            Serial.println("Right position set.");
        },
        "Usage: rp <position_ticks> \n Resets encoders and sets right wheel to target position in ticks."
        });

            CommandInterpreter::registerCommand({"pp", [](const std::string* args)
        {
            long targetPos = std::stol(*args);
            DriveBase::motorPivotFront.setTargetPosition(targetPos * DriveBase::motorPivotFront.getEncoder()->getCPR() / 360); // Convert degrees to ticks
            DriveBase::motorPivotRear.setTargetPosition(targetPos * DriveBase::motorPivotRear.getEncoder()->getCPR() / 360); // Convert degrees to ticks
            DriveBase::motorPivotFront.enableRawPositionControl(true);
            DriveBase::motorPivotRear.enableRawPositionControl(true);
            DriveBase::motorPivotFront.enableSpeedControl(false);
            DriveBase::motorPivotRear.enableSpeedControl(false);
            DriveBase::driveMode = DriveBase::DriveMode::HEADING_CONTROL;
            Serial.print("Set target position to: ");
            Serial.println(targetPos);
        },
        "Usage: targetPivPos <position_ticks> \n Sets the target position for pivot motors in degrees."
        });

        CommandInterpreter::registerCommand({"resetEncoders", [](const std::string* args)
        {
            DriveBase::resetEncoders();
            Serial.println("Encoders reset.");
        },
        "Usage: resetEncoders \n Resets all drivebase encoders to zero."
        });

        CommandInterpreter::registerCommand({"stopAllMotors", [](const std::string* args)
        {
            DriveBase::motorLeft.coast();
            DriveBase::motorRight.coast();
            DriveBase::motorPivotFront.coast();
            DriveBase::motorPivotRear.coast();
            Serial.println("All motors stopped.");
        },
        "Usage: stopAllMotors \n Stops all drivebase motors."
        });

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
        Serial.print("Left Wheel Encoder: \t");
        DriveBase::motorLeft.getEncoder()->printStatus();
        Serial.print("Right Wheel Encoder:\t");
        DriveBase::motorRight.getEncoder()->printStatus();
        Serial.print("Front Pivot Encoder:\t");
        DriveBase::motorPivotFront.getEncoder()->printStatus(); 
        Serial.print("Rear Pivot Encoder:\t");
        DriveBase::motorPivotRear.getEncoder()->printStatus();
        Serial.println();

        // Serial.print("Left Motor Speed: ");
        // Serial.println(DriveBase::motorLeft.getCurrentSpeed()/4096);
        // Serial.print("Right Motor Speed: ");
        // Serial.println(DriveBase::motorRight.getCurrentSpeed()/4096);
        // Serial.print("Front Pivot Motor Speed: ");
        // Serial.println(DriveBase::motorPivotFront.getCurrentSpeed()/4096);
        // Serial.print("Rear Pivot Motor Speed: ");
        // Serial.println(DriveBase::motorPivotRear.getCurrentSpeed()/4096);

        Serial.print(DriveBase::imu.getHeading());
        Serial.println(" deg");
        
        lastPrintTime = currentTime;
    }
}
