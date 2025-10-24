#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Encoders.h"
#include "IMU.h"

#include "CommandInterpretter.h"
#include "RobotMap.h"
#include <Motors.h>

void interpretCommands();
void printDebug();
// Global encoder objects
Encoder encoderFL(RobotMap::ENC_FLA, RobotMap::ENC_FLB);
Encoder encoderFR(RobotMap::ENC_FRA, RobotMap::ENC_FRB);
Encoder encoderBL(RobotMap::ENC_BLA, RobotMap::ENC_BLB);
Encoder encoderBR(RobotMap::ENC_BRA, RobotMap::ENC_BRB);

Motor motorFL(RobotMap::ENC_FLA, RobotMap::MOTOR_FLB, &encoderFL);
Motor motorFR(RobotMap::ENC_FRA, RobotMap::MOTOR_FRB, &encoderFR);
Motor motorBL(RobotMap::ENC_BLA, RobotMap::MOTOR_BLB, &encoderBL);
Motor motorBR(RobotMap::ENC_BRA, RobotMap::MOTOR_BRB, &encoderBR);

IMUController imuController;

void processSerialCommand(String command);

void setup() {
    Serial.begin(9600);

    unsigned long _t0 = millis();
    while (!Serial && (millis() - _t0 < 2000)) { }

    Wire.begin();

    // Attach interrupts for all encoders
    attachInterrupt(digitalPinToInterrupt(encoderFL.getPinA()), []()
                    { encoderFL.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderFR.getPinA()), []()
                    { encoderFR.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBL.getPinA()), []()
                    { encoderBL.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBR.getPinA()), []()
                    { encoderBR.updateCount(); }, CHANGE);

    motorFL.coast();
    motorFR.coast();
    motorBL.coast();
    motorBR.coast();

    Serial.println("Setup complete - Motor and Encoders ready!");

    CommandInterpreter::begin();
    CommandInterpreter::registerCommand({"ping", [](const std::string* args)
    {
        Serial.println("pong");
    },
    "Usage: ping ## \n Sends 'pong' response. Optionally specify number of times to respond."
    });
}

void loop()
{
    CommandInterpreter::periodic();

    imuController.update();
}

