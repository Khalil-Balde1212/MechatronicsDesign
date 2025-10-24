#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "MotorController.h"
#include "Encoders.h"
#include "IMU.h"

#include "CommandInterpretter.h"

void interpretCommands();
void printDebug();
// Global encoder objects
Encoder encoderFL(RobotMap::ENC_FLA, RobotMap::ENC_FLB);
Encoder encoderFR(RobotMap::ENC_FRA, RobotMap::ENC_FRB);
Encoder encoderBL(RobotMap::ENC_BLA, RobotMap::ENC_BLB);
Encoder encoderBR(RobotMap::ENC_BRA, RobotMap::ENC_BRB);
MotorController motorController;
IMUController imuController;

void processSerialCommand(String command);

void setup() {
    Serial.begin(9600);

    unsigned long _t0 = millis();
    while (!Serial && (millis() - _t0 < 2000)) { }

    Wire.begin();
    
    motorController.begin();

    encoderBR.setInverted(true);
    motorBR.setInverted(true);

    // Attach interrupts for all encoders
    attachInterrupt(digitalPinToInterrupt(encoderFL.getPinA()), []()
                    { encoderFL.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderFR.getPinA()), []()
                    { encoderFR.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBL.getPinA()), []()
                    { encoderBL.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBR.getPinA()), []()
                    { encoderBR.updateCount(); }, CHANGE);

    // // motorFL.stop();
    // // motorFR.stop();
    // // motorBL.stop();
    // motorBR.coast();

    Serial.println("Setup complete - Motor and Encoders ready!");

    CommandInterpreter::begin();
    CommandInterpreter::registerCommand({"ping", [](const std::string* args)
    {
        int count;
        if (args == nullptr)
            count = 1;
        else
            count = atoi(args[0].c_str());
            
        for (int i = 0; i < count; ++i)
            Serial.println("pong");
    },
    "Usage: ping ## \n Sends 'pong' response. Optionally specify number of times to respond."
    });
}

void loop()
{
    CommandInterpreter::periodic();
}

void printDebug()
{
    // Print status every second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000)
    {
        lastPrint = millis();

        // Only print back right motor info
        Serial.print("BR Encoder - Position: ");
        Serial.print(encoderBR.getCount());
        Serial.print(" ticks, Speed: ");
        Serial.print(encoderBR.getRPS(), 2);
        Serial.print(" RPS | ");

        Serial.print("Motor - Mode: ");
        Serial.print(motorBR.getControlMode());

        // Show different info based on control mode
        if (strcmp(motorBR.getControlMode(), "Position") == 0)
        {
            Serial.print(", Pos Target: ");
            Serial.print(motorBR.getTargetPosition());
            Serial.print(", Error: ");
            Serial.print(motorBR.getTargetPosition() - encoderBR.getCount());
        }
        Serial.println("==============\n");
    } else if (command == "help") {
        Serial.println("\n=== COMMANDS ===");
        Serial.println("Motors: rl/rr/ra/rb/rc/rd/rf/rt");
        Serial.println("PID: kp/ki/kd");
        Serial.println("IMU: ir ia is ic");
        Serial.println("Other: x status help");
        Serial.println("================\n");
    } else if (command.length() > 0) {
        Serial.println("Unknown (type 'help')");
    }
}

