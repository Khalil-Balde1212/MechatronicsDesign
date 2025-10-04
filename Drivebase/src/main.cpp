#include <Arduino.h>
#include "RobotMap.h"
#include "Encoders.h"
#include "Motors.h"

void interpretCommands();
void printDebug();
// Global encoder objects
Encoder encoderFL(RobotMap::ENC_FLA, RobotMap::ENC_FLB);
Encoder encoderFR(RobotMap::ENC_FRA, RobotMap::ENC_FRB);
Encoder encoderBL(RobotMap::ENC_BLA, RobotMap::ENC_BLB);
Encoder encoderBR(RobotMap::ENC_BRA, RobotMap::ENC_BRB);

// Motor motorFL(RobotMap::MOTOR_FLA, RobotMap::MOTOR_FLB);
// Motor motorFR(RobotMap::MOTOR_FRA, RobotMap::MOTOR_FRB);
// Motor motorBL(RobotMap::MOTOR_BLA, RobotMap::MOTOR_BLB);
Motor motorBR(RobotMap::MOTOR_BRA, RobotMap::MOTOR_BRB, &encoderBR);

void setup()
{
    Serial.begin(9600);

    // Initialize motor PWM driver
    Motor::initializePWM();
    Serial.println("Motor PWM initialized");

    // Initialize all encoders
    encoderFL.begin();
    encoderFR.begin();
    encoderBL.begin();
    encoderBR.begin();

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

    // motorFL.stop();
    // motorFR.stop();
    // motorBL.stop();
    motorBR.coast();

    Serial.println("Setup complete - Motor and Encoders ready!");

    // Enable position control for back right motor
    motorBR.enableRawPositionControl(true);

    motorBR.enableRawPositionControl(true);
    motorBR.setPositionPID(100, 0, 0);
    motorBR.setTargetPosition(2880); // Example: Move to 1 rotation (1440 ticks)
}

void loop()
{
    // Update back right motor control loop
    motorBR.updateControl();
    // motorBR.setSpeed(motorBR.calculatePID(motorBR.positionPID, encoderBR.getCount()));
    interpretCommands();

    // printDebug();
    Serial.print("> pos:");
    Serial.println(encoderBR.getCount());
}

void interpretCommands()
{
    // Check for serial input
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove whitespace

        if (input.equalsIgnoreCase("reset"))
        {
            encoderBR.reset();
            Serial.println("Encoder reset to 0");
        }
        else if (input.equalsIgnoreCase("stop"))
        {
            motorBR.enableRawPositionControl(false);
            motorBR.enableSpeedControl(false);
            motorBR.coast();
            Serial.println("All control disabled - motor stopped (manual mode)");
        }
        else if (input.startsWith("s") || input.startsWith("S"))
        {
            // Speed control command (e.g., "s2.5", "s-1.0", "s0")
            String speedStr = input.substring(1); // Remove 's' prefix
            float targetSpeed = speedStr.toFloat();

            if (speedStr.equals("0") || targetSpeed != 0.0 || speedStr.indexOf("0") >= 0)
            {
                motorBR.enableSpeedControl(true);
                motorBR.setTargetSpeed(targetSpeed);
                Serial.print("Speed control enabled - Target: ");
                Serial.print(targetSpeed, 1);
                Serial.println(" RPS");
            }
            else
            {
                Serial.println("Invalid speed command. Use format: s2.5, s-1.0, s0");
            }
        }
        else if (input.startsWith("kp"))
        {
            float value = input.substring(2).toFloat();
            motorBR.setSpeedPID(value, motorBR.positionPID.ki, motorBR.positionPID.kd);
            Serial.print("Set kp to ");
            Serial.println(value);
        }
        else if (input.startsWith("ki"))
        {
            float value = input.substring(2).toFloat();
            motorBR.setSpeedPID(motorBR.positionPID.kp, value, motorBR.positionPID.kd);
            Serial.print("Set ki to ");
            Serial.println(value);
        }
        else if (input.startsWith("kd"))
        {
            float value = input.substring(2).toFloat();
            motorBR.setSpeedPID(motorBR.positionPID.kp, motorBR.positionPID.ki, value);
            Serial.print("Set kd to ");
            Serial.println(value);
        }
        else
        {
            // Try to parse as position number
            long newPosition = input.toInt();
            if (input.equals("0") || newPosition != 0)
            {
                motorBR.enableRawPositionControl(true);
                motorBR.setTargetPosition(newPosition);
                Serial.print("Position control enabled - Target: ");
                Serial.print(newPosition);
                Serial.println(" ticks");
            }
            else
            {
                Serial.println("Invalid command. Examples:");
                Serial.println("  Position: 1440, -720, 0");
                Serial.println("  Speed: s2.5, s-1.0, s0");
                Serial.println("  Utility: reset, stop");
                Serial.println("  PID: kp5.0 ki0.1 kd1.0");
            }
        }
    }
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
        else if (strcmp(motorBR.getControlMode(), "Speed") == 0)
        {
            Serial.print(", Speed Target: ");
            Serial.print(motorBR.getTargetSpeed(), 1);
            Serial.print(" RPS, Error: ");
            Serial.print(motorBR.getTargetSpeed() - encoderBR.getRPS(), 2);
        }

        // Show if motor has reached target
        if (motorBR.isAtTarget())
        {
            Serial.print(" ✓ AT TARGET");
        }
        else
        {
            Serial.print(" → Moving");
        }

        Serial.println();
    }
}



// void printStatus()
// {
//     Serial.println("=== System Status ===");
//     Serial.print("PID Mode: ");
//     Serial.println(currentPIDMode == POSITION_PID ? "POSITION" : "SPEED");
//     Serial.print("Auto: ");
//     Serial.println(navigation.isAutoActive() ? "ENABLED" : "DISABLED");
//     Serial.print("Moving: ");
//     Serial.println(navigation.isMoving() ? "YES" : "NO");
//     Serial.print("Minimum distance (cm): ");
//     Serial.println(navigation.getMinDistance());
//     Serial.print("Sensor detail output: ");
//     Serial.println(printSensorDetails ? "ENABLED" : "DISABLED");

//     Serial.println("--- Sensor Status ---");
//     for (int i = 0; i < TOF::SENSOR_COUNT; ++i)
//     {
//         Serial.print("Sensor ");
//         Serial.print(i);
//         Serial.print(": ");
//         printSensorLine("", i);
//     }

//     Serial.println("--- Motor Status ---");
//     Serial.print("FL: Current=");
//     Serial.print(Encoders::getRotationsFL());
//     Serial.print(" rotations, Setpoint=");
//     Serial.print(motorController.getSetpointRotationsFL());
//     Serial.println(" rotations");

//     Serial.print("FR: Current=");
//     Serial.print(Encoders::getRotationsFR());
//     Serial.print(" rotations, Setpoint=");
//     Serial.print(motorController.getSetpointRotationsFR());
//     Serial.println(" rotations");

//     Serial.print("BL: Current=");
//     Serial.print(Encoders::getRotationsBL());
//     Serial.print(" rotations, Setpoint=");
//     Serial.print(motorController.getSetpointRotationsBL());
//     Serial.println(" rotations");

//     Serial.print("BR: Current=");
//     Serial.print(Encoders::getRotationsBR());
//     Serial.print(" rotations, Setpoint=");
//     Serial.print(motorController.getSetpointRotationsBR());
//     Serial.println(" rotations");

//     Serial.print("PID Gains: Kp=");
//     Serial.print(motorController.getKp());
//     Serial.print(", Ki=");
//     Serial.print(motorController.getKi());
//     Serial.print(", Kd=");
//     Serial.println(motorController.getKd());
//     Serial.println("====================");
// }

// void printSensorLine(const char *label, int sensorIndex)
// {
//     float distance = tofSensors.getFilteredDistanceCM(sensorIndex);
//     bool timeout = tofSensors.sensorTimeout(sensorIndex) || distance <= 0.0f;

//     Serial.print(label);
//     if (timeout)
//     {
//         Serial.println("TIMEOUT");
//     }
//     else
//     {
//         Serial.print(distance);
//         Serial.println(" cm");
//     }
// }