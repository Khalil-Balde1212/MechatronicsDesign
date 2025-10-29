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


TOF::TOFSensors tofSensors;


unsigned long lastIMUPublish = 0;
unsigned long lastEncoderPublish = 0;
unsigned long lastTOFPublish = 0;


const unsigned long IMU_INTERVAL = 10;
const unsigned long ENCODER_INTERVAL = 20;
const unsigned long TOF_INTERVAL = 100;

String serialCmd = "";
float cmd_vx = 0.0f;
float cmd_vy = 0.0f;
float cmd_yaw = 0.0f;
bool cmd_received = false;


void parseIncomingCommand(const String& line) {
    // Expected: {"cmd":[vx, vy, yaw]}
    int start = line.indexOf("[");
    int mid1 = line.indexOf(",", start);
    int mid2 = line.indexOf(",", mid1 + 1);
    int end = line.indexOf("]", mid2);

    if (start >= 0 && mid1 > start && mid2 > mid1 && end > mid2) {
        cmd_vx = line.substring(start + 1, mid1).toFloat();
        cmd_vy = line.substring(mid1 + 1, mid2).toFloat();
        cmd_yaw = line.substring(mid2 + 1, end).toFloat();
        cmd_received = true;
        Serial.println("{\"ack\":true}");
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!tofSensors.initialize(RobotMap::TOF_XSHUT_PINS, RobotMap::TOF_ADDRESSES, A4, A5)) {
        Serial.println("{\"error\":\"Failed to configure TOF sensors\"}");
    } else if (!tofSensors.begin()) {
        Serial.println("{\"error\":\"TOF sensor initialization encountered errors\"}");
    } else {
        Serial.println("{\"status\":\"TOF sensors initialized successfully\"}");
    }

    CommandInterpreter::begin();
    DriveBase::begin();
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

void loop() {
    CommandInterpreter::periodic();
    DriveBase::update();

    // ROS2 COMMAND INPUT (non-blocking)
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            parseIncomingCommand(serialCmd);
            serialCmd = "";
        } else {
            serialCmd += c;
        }
    }

    // APPLY ROS2 COMMAND IF AVAILABLE
    if (cmd_received) {
        float headingError = DriveBase::imu.getHeading() - cmd_yaw;
        float correction = headingError / 180.0f * 4095 * 15;

        DriveBase::motorLeft.setSpeed(4096 + correction);
        DriveBase::motorRight.setSpeed(4096 - correction);

        float angle = atan2(cmd_vy, cmd_vx) * 180.0f / PI;
        DriveBase::motorPivotLeft.setTargetPosition(angle);
        DriveBase::motorPivotRight.setTargetPosition(angle);
    }

    unsigned long now = millis();

    if (now - lastIMUPublish >= IMU_INTERVAL) {
        lastIMUPublish = now;
        Serial.print("{\"imu\":[");
        Serial.print(DriveBase::imu.getRoll(), 4); Serial.print(",");
        Serial.print(DriveBase::imu.getPitch(), 4); Serial.print(",");
        Serial.print(DriveBase::imu.getHeading(), 4);
        Serial.println("]}");
    }

    if (now - lastEncoderPublish >= ENCODER_INTERVAL) {
        lastEncoderPublish = now;
        Serial.print("{\"enc\":[");
        Serial.print(DriveBase::motorLeft.getEncoder()->getCount()); Serial.print(",");
        Serial.print(DriveBase::motorRight.getEncoder()->getCount()); Serial.print(",");
        Serial.print(DriveBase::encoderPivotLeft.getRotations(), 4); Serial.print(",");
        Serial.print(DriveBase::encoderPivotRight.getRotations(), 4);
        Serial.println("]}");
    }

    if (now - lastTOFPublish >= TOF_INTERVAL) {
        lastTOFPublish = now;
        Serial.print("{\"tof\":[");
        Serial.print(tofSensors.getFilteredDistanceCM(RobotMap::TOF_RIGHT_ID)); Serial.print(",");
        Serial.print(tofSensors.getFilteredDistanceCM(RobotMap::TOF_LEFT_ID)); Serial.print(",");
        Serial.print(tofSensors.getFilteredDistanceCM(RobotMap::TOF_FRONT_ID));
        Serial.println("]}");
    }
}