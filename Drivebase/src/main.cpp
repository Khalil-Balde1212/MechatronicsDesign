#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "MotorController.h"
#include "Encoders.h"
#include "IMU.h"

MotorController motorController;
IMUController imuController;
void processSerialCommand(String command);

void setup() {
    Serial.begin(9600);

    // --- IMU/Nano 33 IoT specifics (from Arduino guidance) ---
    // Let native USB enumerate so you see startup messages.
    unsigned long _t0 = millis();
    while (!Serial && (millis() - _t0 < 2000)) { /* wait up to ~2s */ }

    // Ensure I2C is up (IMU is I2C). Safe even if library handles it.
    Wire.begin();
    // ----------------------------------------------------------

    motorController.begin();

    // IMU init (halt on failure as in Arduino examples)
    if (!imuController.begin()) {
        while (1) { /* Halt execution if IMU fails */ }
    }
}

void loop() {
    // Keep IMU readings fresh each loop
    imuController.update();

    motorController.updateAllPID();
  
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }
  
    // Print status every 500ms
    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        // Encoders::printRotations();

        // Print IMU data if initialized, using library units:
        // Gyro -> deg/s, Accel -> g (per Arduino_LSM6DS3 docs)
        if (imuController.isInitialized()) {
            Serial.print("[IMU] ");
            imuController.printGyroData();   // deg/s
            imuController.printAccelData();  // g
        }

        last_print = millis();
    }
}

// Function to process serial commands
void processSerialCommand(String command) {
    command.trim();
    
    if (command.length() >= 2) {
        char firstChar = command.charAt(0);
        char secondChar = command.charAt(1);
        
        if (firstChar == 'r') {
            // Position commands
            float value = command.substring(2).toFloat();
            
            switch (secondChar) {
                case 'l': {
                    motorController.setPositionFL(value);
                    motorController.setPositionBL(value);
                    Serial.print("Left motors set to ");
                    Serial.print(value);
                    Serial.println(" rotations");
                    break;
                }
                case 'r': {
                    motorController.setPositionFR(value);
                    motorController.setPositionBR(value);
                    Serial.print("Right motors set to ");
                    Serial.print(value);
                    Serial.println(" rotations");
                    break;
                }
                case 'a': {
                    motorController.setPositionFR(value);
                    Serial.print("Front Right motor set to ");
                    Serial.print(value);
                    Serial.println(" rotations");
                    break;
                }
                case 'b': {
                    motorController.setPositionBR(value);
                    Serial.print("Back Right motor set to ");
                    Serial.print(value);
                    Serial.println(" rotations");
                    break;
                }
                case 'c': {
                    motorController.setPositionFL(value);
                    Serial.print("Front Left motor set to ");
                    Serial.print(value);
                    Serial.println(" rotations");
                    break;
                }
                case 'd': {
                    motorController.setPositionBL(value);
                    Serial.print("Back Left motor set to ");
                    Serial.print(value);
                    Serial.println(" rotations");
                    break;
                }
                case 'f': {
                    motorController.setPositionFL(motorController.getSetpointRotationsFL() + value);
                    motorController.setPositionFR(motorController.getSetpointRotationsFR() + value);
                    motorController.setPositionBL(motorController.getSetpointRotationsBL() + value);
                    motorController.setPositionBR(motorController.getSetpointRotationsBR() + value);
                    Serial.print("All motors set to ");
                    Serial.print(value);
                    Serial.println(" rotations (forward)");
                    break;
                }
                case 't': {
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
                }
                default: {
                    Serial.println("Unknown position command");
                    break;
                }
            }
        } else if (firstChar == 'k') {
            // PID gain commands
            float value = command.substring(2).toFloat();
            
            switch (secondChar) {
                case 'p': {
                    motorController.setKp(value);
                    Serial.print("Kp set to ");
                    Serial.println(value);
                    break;
                }
                case 'i': {
                    motorController.setKi(value);
                    Serial.print("Ki set to ");
                    Serial.println(value);
                    break;
                }
                case 'd': {
                    motorController.setKd(value);
                    Serial.print("Kd set to ");
                    Serial.println(value);
                    break;
                }
                default: {
                    Serial.println("Unknown PID gain command (use kp, ki, or kd)");
                    break;
                }
            }
        }
    } else if (command == "x") {
        // Reset all encoder positions
        Encoders::resetAll();
        motorController.resetPIDVariables();
        Serial.println("All positions reset to 0");
    } else if (command == "status") {
        // Print current status
        Serial.println("=== Current Status ===");
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
    } else {
        Serial.println("Unknown command");
    }
}
