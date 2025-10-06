#include <Arduino.h>
#include "RobotMap.h"
#include "Encoders.h"
#include "Motors.h"
#include "Drivebase.h"

void setup()
{
    Serial.begin(9600);
    DriveBase::begin();
}

void loop()
{
    DriveBase::update();
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