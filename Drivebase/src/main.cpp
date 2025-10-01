#include <Arduino.h>
#include <Wire.h>

#include "MotorController.h"
#include "Encoders.h"
#include "tof_cameras.h"
#include "Navigation.h"

// Pin definitions for TOF sensors (XSHUT order: BR, BL, FL, FR)
const int TOF_XSHUT_PINS[TOF::SENSOR_COUNT] = {5, 6, 12, 13};
const uint8_t TOF_ADDRESSES[TOF::SENSOR_COUNT] = {0x30, 0x31, 0x32, 0x33};

TOF::TOFSensors tofSensors;
MotorController motorController;
Navigation::Navigation navigation(motorController, tofSensors);
bool printSensorDetails = false;

void processSerialCommand(String command);
void printStatus();
void printSensorLine(const char *label, int sensorIndex);

void setup() {
  Serial.begin(9600);
  Serial.println("Starting initialization...");

  motorController.begin();
  Serial.println("Motor controller initialized");

  if (!tofSensors.initialize(TOF_XSHUT_PINS, TOF_ADDRESSES, 0, 0)) {
    Serial.println("Failed to configure TOF sensors");
  } else if (!tofSensors.begin()) {
    Serial.println("TOF sensor initialization encountered errors");
  } else {
    Serial.println("TOF sensors initialized successfully");
  }

  // navigation.begin();

  
}

void loop() {
  motorController.updateAllPID();

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processSerialCommand(command);
  }

  navigation.update();

  //print outs
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    Serial.println("\n=== Navigation ===");
    Serial.print("Auto: ");
    Serial.println(navigation.isAutoActive() ? "ENABLED" : "DISABLED");
    Serial.print("Moving: ");
    Serial.println(navigation.isMoving() ? "YES" : "NO");
    Serial.print("Minimum distance (cm): ");
    Serial.println(navigation.getMinDistance());
    Serial.print("Sensor details: ");
    Serial.println(printSensorDetails ? "ENABLED" : "DISABLED");

    // Print encoder rotations per second
    Serial.println("\n=== Motor Speeds ===");
    Encoders::printRPS();

    if (printSensorDetails) {
      for (int i = 0; i < TOF::SENSOR_COUNT; ++i) {
        float distance = tofSensors.getFilteredDistanceCM(i);
        bool timeout = tofSensors.sensorTimeout(i) || distance <= 0.0f;
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" (XSHUT pin ");
        Serial.print(TOF_XSHUT_PINS[i]);
        Serial.print(") : ");
        if (timeout) {
          Serial.println("TIMEOUT");
        } else {
          Serial.print(distance);
          Serial.println(" cm");
        }
      }
    }

    Serial.println("===================\n");
    lastPrint = millis();
  }
}

void processSerialCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  if (command.equalsIgnoreCase("stop")) {
    navigation.stop();
    Serial.println("Navigation stopped");
    return;
  }

  if (command.equalsIgnoreCase("auto")) {
    navigation.startAuto();
    Serial.println("Auto navigation started");
    return;
  }

  if (command.equalsIgnoreCase("manual")) {
    navigation.stopAuto();
    Serial.println("Auto navigation stopped");
    return;
  }

  if (command.equalsIgnoreCase("status")) {
    printStatus();
    return;
  }

  if (command.equalsIgnoreCase("sensors")) {
    printSensorDetails = !printSensorDetails;
    Serial.print("Sensor detail printing ");
    Serial.println(printSensorDetails ? "ENABLED" : "DISABLED");
    return;
  }

  if (command.equalsIgnoreCase("x")) {
    navigation.stop();
    Encoders::resetAll();
    motorController.resetPIDVariables();
    Serial.println("All positions reset to 0");
    return;
  }

  if (command.length() >= 2) {
    char firstChar = command.charAt(0);
    char secondChar = command.charAt(1);

    if (firstChar == 'n') {
      float value = command.substring(2).toFloat();

      switch (secondChar) {
        case 'f':
          navigation.moveForward(value);
          Serial.print("Moving forward ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        case 'b':
          navigation.moveBackward(value);
          Serial.print("Moving backward ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        case 'l':
          navigation.turnLeft(value);
          Serial.print("Turning left ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        case 'r':
          navigation.turnRight(value);
          Serial.print("Turning right ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        default:
          Serial.println("Unknown navigation command (use nf, nb, nl, nr)");
          break;
      }
      return;
    }

    if (firstChar == 'r') {
      float value = command.substring(2).toFloat();

      switch (secondChar) {
        case 'l':
          motorController.setPositionFL(value);
          motorController.setPositionBL(value);
          Serial.print("Left motors set to ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        case 'r':
          motorController.setPositionFR(value);
          motorController.setPositionBR(value);
          Serial.print("Right motors set to ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        case 'a':
          motorController.setPositionFR(value);
          Serial.print("Front Right motor set to ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        case 'b':
          motorController.setPositionBR(value);
          Serial.print("Back Right motor set to ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        case 'c':
          motorController.setPositionFL(value);
          Serial.print("Front Left motor set to ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        case 'd':
          motorController.setPositionBL(value);
          Serial.print("Back Left motor set to ");
          Serial.print(value);
          Serial.println(" rotations");
          break;
        case 'f':
          motorController.setPositionFL(motorController.getSetpointRotationsFL() + value);
          motorController.setPositionFR(motorController.getSetpointRotationsFR() + value);
          motorController.setPositionBL(motorController.getSetpointRotationsBL() + value);
          motorController.setPositionBR(motorController.getSetpointRotationsBR() + value);
          Serial.print("All motors set to ");
          Serial.print(value);
          Serial.println(" rotations (forward)");
          break;
        case 't':
          motorController.setPositionFL(motorController.getSetpointRotationsFL() + value);
          motorController.setPositionBL(motorController.getSetpointRotationsBL() + value);
          motorController.setPositionFR(motorController.getSetpointRotationsFR() - value);
          motorController.setPositionBR(motorController.getSetpointRotationsBR() - value);
          Serial.print("Turn command: Left side ");
          Serial.print(value);
          Serial.print(" rotations, Right side ");
          Serial.print(-value);
          Serial.println(" rotations");
          break;
        default:
          Serial.println("Unknown position command");
          break;
      }
      return;
    }

    if (firstChar == 'k') {
      float value = command.substring(2).toFloat();

      switch (secondChar) {
        case 'p':
          motorController.setKp(value);
          Serial.print("Kp set to ");
          Serial.println(value);
          break;
        case 'i':
          motorController.setKi(value);
          Serial.print("Ki set to ");
          Serial.println(value);
          break;
        case 'd':
          motorController.setKd(value);
          Serial.print("Kd set to ");
          Serial.println(value);
          break;
        default:
          Serial.println("Unknown PID gain command (use kp, ki, or kd)");
          break;
      }
      return;
    }
  }

  Serial.println("Unknown command");
}

void printStatus() {
  Serial.println("=== System Status ===");
  Serial.print("Auto: ");
  Serial.println(navigation.isAutoActive() ? "ENABLED" : "DISABLED");
  Serial.print("Moving: ");
  Serial.println(navigation.isMoving() ? "YES" : "NO");
  Serial.print("Minimum distance (cm): ");
  Serial.println(navigation.getMinDistance());
  Serial.print("Sensor detail output: ");
  Serial.println(printSensorDetails ? "ENABLED" : "DISABLED");

  Serial.println("--- Sensor Status ---");
  for (int i = 0; i < TOF::SENSOR_COUNT; ++i) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    printSensorLine("", i);
  }

  Serial.println("--- Motor Status ---");
  Serial.print("FL: Current=");
  Serial.print(Encoders::getRotationsFL());
  Serial.print(" rotations, Setpoint=");
  Serial.print(motorController.getSetpointRotationsFL());
  Serial.println(" rotations");

  Serial.print("FR: Current=");
  Serial.print(Encoders::getRotationsFR());
  Serial.print(" rotations, Setpoint=");
  Serial.print(motorController.getSetpointRotationsFR());
  Serial.println(" rotations");

  Serial.print("BL: Current=");
  Serial.print(Encoders::getRotationsBL());
  Serial.print(" rotations, Setpoint=");
  Serial.print(motorController.getSetpointRotationsBL());
  Serial.println(" rotations");

  Serial.print("BR: Current=");
  Serial.print(Encoders::getRotationsBR());
  Serial.print(" rotations, Setpoint=");
  Serial.print(motorController.getSetpointRotationsBR());
  Serial.println(" rotations");

  Serial.print("PID Gains: Kp=");
  Serial.print(motorController.getKp());
  Serial.print(", Ki=");
  Serial.print(motorController.getKi());
  Serial.print(", Kd=");
  Serial.println(motorController.getKd());
  Serial.println("====================");
}

void printSensorLine(const char *label, int sensorIndex) {
  float distance = tofSensors.getFilteredDistanceCM(sensorIndex);
  bool timeout = tofSensors.sensorTimeout(sensorIndex) || distance <= 0.0f;

  Serial.print(label);
  if (timeout) {
    Serial.println("TIMEOUT");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }
}