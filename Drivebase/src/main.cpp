#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Encoders.h"
#include "IMU.h"

#include "CommandInterpretter.h"
#include "RobotMap.h"
#include <Motors.h>

#include "drivebase/Drivebase.h"

#include <tof_cameras.h>

// Pin Definitions for TOF Sensors

TOF::TOFSensors tofSensors;

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    if (!tofSensors.initialize(RobotMap::TOF_XSHUT_PINS, RobotMap::TOF_ADDRESSES, A4, A5))
    {
        Serial.println("Failed to configure TOF sensors");
    }
    else if (!tofSensors.begin())
    {
        Serial.println("TOF sensor initialization encountered errors");
    }
    else
    {
        Serial.println("TOF sensors initialized successfully");
    }

    CommandInterpreter::begin();
    DriveBase::begin();

    // Calibrate IMU gyro at startup (robot must be stationary)
    Serial.println("Calibrating IMU gyro, please keep robot still...");
    DriveBase::imu.calibrateGyro();
    Serial.println("IMU gyro calibration complete.");

    // Disable the built-in heading PID to use motor's position control
    DriveBase::enableHeadingPID(false);

    CommandInterpreter::registerCommand({"rawRightSpeed", [](const std::string *args)
                                         {
                                             Serial.print("Setting raw speed for pivot motor to: ");
                                             Serial.println(args->c_str());
                                             double speed = std::stod(args[0]);
                                             DriveBase::setRawSpeedRight(speed);
                                         },
                                         "Usage: rawRightSpeed \n Enables raw position control for pivot motors."});

    CommandInterpreter::registerCommand({"rawLeftSpeed", [](const std::string *args)
                                         {
                                             Serial.print("Setting raw left speed for pivot motor to: ");
                                             Serial.println(args->c_str());
                                             double speed = std::stod(args[0]);
                                             DriveBase::setRawSpeedLeft(speed);
                                         },
                                         "Usage: rawLeftSpeed \n Enables raw position control for pivot motors."});

    CommandInterpreter::registerCommand({"rawRightPivotSpeed", [](const std::string *args)
                                         {
                                             Serial.print("Setting raw speed for pivot motor to: ");
                                             Serial.println(args->c_str());
                                             double speed = std::stod(args[0]);
                                             DriveBase::setRawSpeedRightPivot(speed);
                                         },
                                         "Usage: rawRightPivotSpeed \n Enables raw position control for pivot motors."});

    CommandInterpreter::registerCommand({"rawLeftPivotSpeed", [](const std::string *args)
                                         {
                                             Serial.print("Setting raw left speed for pivot motor to: ");
                                             Serial.println(args->c_str());
                                             double speed = std::stod(args[0]);
                                             DriveBase::setRawSpeedLeftPivot(speed);
                                         },
                                         "Usage: rawLeftPivotSpeed \n Enables raw position control for pivot motors."});

    CommandInterpreter::registerCommand({"rawPivotSpeed", [](const std::string *args)
                                         {
                                             Serial.print("Setting raw speed for pivot motor to: ");
                                             Serial.println(args->c_str());
                                             double speed = std::stod(args[0]);
                                             DriveBase::setRawSpeedRightPivot(speed);
                                             DriveBase::setRawSpeedLeftPivot(speed);
                                         },
                                         "Usage: rawPivotSpeed \n Enables raw position control for pivot motors."});

    CommandInterpreter::registerCommand({"lp", [](const std::string *args)
                                         {
                                             DriveBase::setPositionLeft(std::stoi(args[0]));
                                             Serial.println("Left position set.");
                                         },
                                         "Usage: lp <position_ticks> \n Resets encoders and sets left wheel to target position in ticks."});

    CommandInterpreter::registerCommand({"rp", [](const std::string *args)
                                         {
                                             DriveBase::setPositionRight(std::stoi(args[0]));
                                             Serial.println("Right position set.");
                                         },
                                         "Usage: rp <position_ticks> \n Resets encoders and sets right wheel to target position in ticks."});

    CommandInterpreter::registerCommand({"pp", [](const std::string *args)
                                         {
                                             long targetPos = std::stol(*args);

                                             // Stop all motors and disable control first!
                                             DriveBase::motorPivotRight.enableRawPositionControl(false);
                                             DriveBase::motorPivotLeft.enableRawPositionControl(false);
                                             DriveBase::motorPivotRight.enableSpeedControl(false);
                                             DriveBase::motorPivotLeft.enableSpeedControl(false);
                                             DriveBase::motorPivotRight.coast();
                                             DriveBase::motorPivotLeft.coast();

                                             // DON'T reset encoders - use absolute positioning from initial calibration
                                             // DriveBase::encoderPivotRight.reset();
                                             // DriveBase::encoderPivotLeft.reset();

                                             long targetTicksRight = targetPos * DriveBase::motorPivotRight.getEncoder()->getCPR() / 360;
                                             long targetTicksLeft = targetPos * DriveBase::motorPivotLeft.getEncoder()->getCPR() / 360;

                                             // Limit targets based on observed mechanical constraints
                                             // Left motor reached 5817 ticks (~362°), right motor limited to ~5728 ticks
                                             const long MAX_RIGHT_TICKS = 5728; // Right motor mechanical limit observed
                                             const long MAX_LEFT_TICKS = 5817;  // Left motor actual capability observed

                                             if (targetTicksRight > MAX_RIGHT_TICKS)
                                             {
                                                 targetTicksRight = MAX_RIGHT_TICKS;
                                                 Serial.println("WARNING: Limiting right motor to observed mechanical limit (5728 ticks)");
                                             }
                                             if (targetTicksLeft > MAX_LEFT_TICKS)
                                             {
                                                 targetTicksLeft = MAX_LEFT_TICKS;
                                                 Serial.println("WARNING: Limiting left motor to observed mechanical limit (5817 ticks)");
                                             }

                                             // Debug output
                                             Serial.print("Target degrees: ");
                                             Serial.println(targetPos);
                                             Serial.print("Right CPR: ");
                                             Serial.println(DriveBase::motorPivotRight.getEncoder()->getCPR());
                                             Serial.print("Left CPR: ");
                                             Serial.println(DriveBase::motorPivotLeft.getEncoder()->getCPR());
                                             Serial.print("Target ticks right: ");
                                             Serial.println(targetTicksRight);
                                             Serial.print("Target ticks left: ");
                                             Serial.println(targetTicksLeft);
                                             Serial.print("Current right position: ");
                                             Serial.println(DriveBase::motorPivotRight.getCurrentPosition());
                                             Serial.print("Current left position: ");
                                             Serial.println(DriveBase::motorPivotLeft.getCurrentPosition());

                                             DriveBase::motorPivotRight.setTargetPosition(targetTicksRight);
                                             DriveBase::motorPivotLeft.setTargetPosition(targetTicksLeft);
                                             DriveBase::motorPivotRight.enableRawPositionControl(true);
                                             DriveBase::motorPivotLeft.enableRawPositionControl(true);
                                             DriveBase::driveMode = DriveBase::DriveMode::HEADING_CONTROL;
                                         },
                                         "Usage: pp <position_degrees> \n Sets the target position for pivot motors in degrees."});

    CommandInterpreter::registerCommand({"testRight", [](const std::string *args)
                                         {
                                             Serial.println("Testing right pivot motor...");
                                             DriveBase::motorPivotRight.enableRawPositionControl(false);
                                             DriveBase::motorPivotRight.enableSpeedControl(false);
                                             DriveBase::motorPivotRight.setSpeed(1500); // Test speed
                                             delay(1000);
                                             DriveBase::motorPivotRight.coast();
                                             Serial.print("Right encoder count: ");
                                             Serial.println(DriveBase::encoderPivotRight.getCount());
                                         },
                                         "Test right pivot motor and encoder"});

    CommandInterpreter::registerCommand({"testLeft", [](const std::string *args)
                                         {
                                             Serial.println("Testing left pivot motor...");
                                             DriveBase::motorPivotLeft.enableRawPositionControl(false);
                                             DriveBase::motorPivotLeft.enableSpeedControl(false);
                                             DriveBase::motorPivotLeft.setSpeed(1500); // Test speed
                                             delay(1000);
                                             DriveBase::motorPivotLeft.coast();
                                             Serial.print("Left encoder count: ");
                                             Serial.println(DriveBase::encoderPivotLeft.getCount());
                                         },
                                         "Test left pivot motor and encoder"});

    CommandInterpreter::registerCommand({"powerTest", [](const std::string *args)
                                         {
                                             Serial.println("Testing motor power (no PID)...");
                                             DriveBase::motorPivotRight.enableRawPositionControl(false);
                                             DriveBase::motorPivotLeft.enableRawPositionControl(false);
                                             DriveBase::motorPivotRight.enableSpeedControl(false);
                                             DriveBase::motorPivotLeft.enableSpeedControl(false);

                                             Serial.println("Right motor forward...");
                                             DriveBase::motorPivotRight.setSpeed(3000); // Higher speed
                                             delay(2000);
                                             DriveBase::motorPivotRight.coast();

                                             Serial.println("Left motor forward...");
                                             DriveBase::motorPivotLeft.setSpeed(3000); // Higher speed
                                             delay(2000);
                                             DriveBase::motorPivotLeft.coast();

                                             Serial.println("Right motor reverse...");
                                             DriveBase::motorPivotRight.setSpeed(-3000); // Higher speed
                                             delay(2000);
                                             DriveBase::motorPivotRight.coast();

                                             Serial.println("Left motor reverse...");
                                             DriveBase::motorPivotLeft.setSpeed(-3000); // Higher speed
                                             delay(2000);
                                             DriveBase::motorPivotLeft.coast();

                                             Serial.print("Final right encoder: ");
                                             Serial.println(DriveBase::encoderPivotRight.getCount());
                                             Serial.print("Final left encoder: ");
                                             Serial.println(DriveBase::encoderPivotLeft.getCount());
                                         },
                                         "Test raw motor power without PID control"});

    CommandInterpreter::registerCommand({"speedCompare", [](const std::string *args)
                                         {
                                             Serial.println("Comparing motor speeds simultaneously...");
                                             DriveBase::motorPivotRight.enableRawPositionControl(false);
                                             DriveBase::motorPivotLeft.enableRawPositionControl(false);
                                             DriveBase::motorPivotRight.enableSpeedControl(false);
                                             DriveBase::motorPivotLeft.enableSpeedControl(false);

                                             // Reset encoders for consistent starting point
                                             DriveBase::encoderPivotRight.reset();
                                             DriveBase::encoderPivotLeft.reset();
                                             delay(100);

                                             Serial.println("Both motors forward at speed 2500...");
                                             DriveBase::motorPivotRight.setSpeed(2500);
                                             DriveBase::motorPivotLeft.setSpeed(2500);
                                             delay(3000);
                                             DriveBase::motorPivotRight.coast();
                                             DriveBase::motorPivotLeft.coast();

                                             Serial.print("Right encoder: ");
                                             Serial.println(DriveBase::encoderPivotRight.getCount());
                                             Serial.print("Left encoder: ");
                                             Serial.println(DriveBase::encoderPivotLeft.getCount());

                                             delay(1000);

                                             Serial.println("Both motors reverse at speed 2500...");
                                             DriveBase::motorPivotRight.setSpeed(-2500);
                                             DriveBase::motorPivotLeft.setSpeed(-2500);
                                             delay(3000);
                                             DriveBase::motorPivotRight.coast();
                                             DriveBase::motorPivotLeft.coast();

                                             Serial.print("Final right encoder: ");
                                             Serial.println(DriveBase::encoderPivotRight.getCount());
                                             Serial.print("Final left encoder: ");
                                             Serial.println(DriveBase::encoderPivotLeft.getCount());
                                         },
                                         "Compare both motors running simultaneously at same speed"});

    CommandInterpreter::registerCommand({"resetEncoders", [](const std::string *args)
                                         {
                                             DriveBase::resetEncoders();
                                             Serial.println("Encoders reset.");
                                         },
                                         "Usage: resetEncoders \n Resets all drivebase encoders to zero."});

    CommandInterpreter::registerCommand({"stopAllMotors", [](const std::string *args)
                                         {
                                             DriveBase::motorLeft.coast();
                                             DriveBase::motorRight.coast();
                                             DriveBase::motorPivotRight.coast();
                                             DriveBase::motorPivotLeft.coast();
                                             Serial.println("All motors stopped.");
                                         },
                                         "Usage: stopAllMotors \n Stops all drivebase motors."});

    CommandInterpreter::registerCommand({"disablePositionControl", [](const std::string *args)
                                         {
                                             DriveBase::motorPivotRight.enableRawPositionControl(false);
                                             DriveBase::motorPivotLeft.enableRawPositionControl(false);
                                             DriveBase::motorPivotRight.coast();
                                             DriveBase::motorPivotLeft.coast();
                                             Serial.println("Position control disabled for pivot motors.");
                                         },
                                         "Usage: disablePositionControl \n Disables position control and stops pivot motors."});
}

static unsigned long lastPrintTime = 0;

void loop()
{
    // Update motor control (this runs the PID for ALL motors)
    CommandInterpreter::periodic();
    DriveBase::update(); // This calls updateControl() for all motors

    float targetHeading = 0.0f;
    float targetyvel = 10.0f;
    float targetxvel = 0.0f;

    // Both motors start at high speed (4096)
    // As the robot veers away from targetHeading, reduce speed of one side to correct
    float headingError = DriveBase::imu.getHeading() - targetHeading;
    float correction = headingError / 180.0f * 4095 * 10; // scale correction

    DriveBase::motorLeft.setSpeed(4096 + correction);  // Reduce left if error positive
    DriveBase::motorRight.setSpeed(4096 - correction); // Reduce right if error negative

    // Calculate angle between target x and y velocity
    float angle = atan2(targetyvel, targetxvel) * 180.0f / PI;
    DriveBase::motorPivotLeft.setTargetPosition(angle);
    DriveBase::motorPivotRight.setTargetPosition(angle);

    // Safety check: Disable position control when motors reach target to prevent runaway
    if (DriveBase::motorPivotRight.isPositionAtTarget())
    {
        DriveBase::motorPivotRight.enableRawPositionControl(false);
        DriveBase::motorPivotRight.coast();
    }
    if (DriveBase::motorPivotLeft.isPositionAtTarget())
    {
        DriveBase::motorPivotLeft.enableRawPositionControl(false);
        DriveBase::motorPivotLeft.coast();
    }
    
        Serial.println(DriveBase::imu.getHeading());
    unsigned long currentTime = millis();
    // Print status every 500ms
    if (currentTime - lastPrintTime >= 500)
    {
        // Serial.print("Left Wheel Encoder: \t");
        // DriveBase::motorLeft.getEncoder()->printStatus();
        // Serial.print("Right Wheel Encoder:\t");
        // DriveBase::motorRight.getEncoder()->printStatus();
        // Serial.print("Right Pivot Encoder:\t");
        // DriveBase::motorPivotRight.getEncoder()->printStatus();
        // Serial.print("Left Pivot Encoder:\t");
        // DriveBase::motorPivotLeft.getEncoder()->printStatus();

        // // Add PID status for pivot motors
        // Serial.println("=== PIVOT MOTOR PID STATUS ===");
        // DriveBase::motorPivotFront.printPIDStatus();
        // DriveBase::motorPivotRear.printPIDStatus();
        // Serial.println("==============================");

        // // Debug PID components for front motor
        // auto& frontPID = DriveBase::motorPivotFront.getPositionPID();
        // Serial.print("Front PID - P: ");
        // Serial.print(frontPID.kp * frontPID.error);
        // Serial.print(", I: ");
        // Serial.print(frontPID.ki * frontPID.integral);
        // Serial.print(", D: ");
        // Serial.print(frontPID.kd * ((frontPID.error - frontPID.lastError) / 0.5));
        // Serial.print(", Total Output: ");
        // Serial.println(frontPID.output);

        // // Debug PID components for rear motor
        // auto& rearPID = DriveBase::motorPivotRear.getPositionPID();
        // Serial.print("Rear PID - P: ");
        // Serial.print(rearPID.kp * rearPID.error);
        // Serial.print(", I: ");
        // Serial.print(rearPID.ki * rearPID.integral);
        // Serial.print(", D: ");
        // Serial.print(rearPID.kd * ((rearPID.error - rearPID.lastError) / 0.5));
        // Serial.print(", Total Output: ");
        // Serial.println(rearPID.output);

        // Serial.println();

        // Serial.print("Left Motor Speed: ");
        // Serial.println(DriveBase::motorLeft.getCurrentSpeed()/4096);
        // Serial.print("Right Motor Speed: ");
        // Serial.println(DriveBase::motorRight.getCurrentSpeed()/4096);
        // Serial.print("Front Pivot Motor Speed: ");
        // Serial.println(DriveBase::motorPivotFront.getCurrentSpeed()/4096);
        // Serial.print("Rear Pivot Motor Speed: ");
        // Serial.println(DriveBase::motorPivotRear.getCurrentSpeed()/4096);

        // Serial.print(DriveBase::imu.getHeading());
        // Serial.println(" deg");

        lastPrintTime = currentTime;

            Serial.print("Distance:\t");
            Serial.print(tofSensors.getFilteredDistanceCM(RobotMap::TOF_RIGHT_ID));
            Serial.print(" cm");
            Serial.print("  \t|\t");
            Serial.print(tofSensors.getFilteredDistanceCM(RobotMap::TOF_LEFT_ID));
            Serial.print(" cm");
            Serial.print("  \t|\t");
            Serial.print(tofSensors.getFilteredDistanceCM(RobotMap::TOF_FRONT_ID));
            Serial.println(" cm");
            // Serial.print("Left Sensor Distance: ");
            // Serial.print(tofSensors.getFilteredDistanceCM(RobotMap::TOF_LEFT_ID));
            // Serial.println(" cm");

            // Serial.print("Front Sensor Distance: \t");
            // Serial.print(tofSensors.getFilteredDistanceCM(RobotMap::TOF_FRONT_ID));
            // Serial.println(" cm");
    }
}
