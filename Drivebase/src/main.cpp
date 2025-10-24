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

    unsigned long _t0 = millis();
    while (!Serial && (millis() - _t0 < 2000)) { }

    Wire.begin();
    
    motorController.begin();

    if (!imuController.begin()) {
        Serial.println("[ERROR] IMU initialization failed!");
        while (1);
    }
    
    // Silent auto-calibration (no prompts)
    delay(2000);  // Wait 2 seconds for sensor to stabilize
    
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int samples = 0;
    
    // Collect 500 samples silently
    for (int i = 0; i < 500; i++) {
        if (IMU.gyroscopeAvailable()) {
            float gx, gy, gz;
            IMU.readGyroscope(gx, gy, gz);
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            samples++;
        }
        delay(10);
    }
    
    // Apply calibration
    if (samples > 0) {
        imuController.setGyroOffsets(gx_sum / samples, gy_sum / samples, gz_sum / samples);
    }
    
    imuController.resetAngles();
    
    Serial.println("[SYSTEM] Ready\n");
}

void loop() {
    imuController.update();
    
    motorController.updateAllPID();
  
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }
  
    static unsigned long last_print = 0;
    if (millis() - last_print > 200) {
        
        if (imuController.isInitialized()) {
            // OPTION 1: Compact one-line format (default)
            Serial.print("[IMU] Roll: ");
            Serial.print(imuController.getRoll(), 1);
            Serial.print("° Pitch: ");
            Serial.print(imuController.getPitch(), 1);
            Serial.print("° Yaw: ");
            Serial.print(imuController.getYaw(), 1);
            Serial.print("° | ω: ");
            Serial.print(imuController.getOmegaX(), 1);
            Serial.print(",");
            Serial.print(imuController.getOmegaY(), 1);
            Serial.print(",");
            Serial.print(imuController.getOmegaZ(), 1);
            Serial.print(" dps | α: ");
            Serial.print(imuController.getAlphaX(), 0);
            Serial.print(",");
            Serial.print(imuController.getAlphaY(), 0);
            Serial.print(",");
            Serial.print(imuController.getAlphaZ(), 0);
            Serial.println(" dps²");
            
            // OPTION 2: Uncomment for full detailed summary (every 2 seconds)
            // static unsigned long last_summary = 0;
            // if (millis() - last_summary > 2000) {
            //     imuController.printCompleteSummary();
            //     last_summary = millis();
            // }
        }

        last_print = millis();
    }
}

void processSerialCommand(String command) {
    command.trim();
    
    if (command.length() >= 2) {
        char firstChar = command.charAt(0);
        char secondChar = command.charAt(1);
        
        if (firstChar == 'r') {
            float value = command.substring(2).toFloat();
            
            switch (secondChar) {
                case 'l':
                    motorController.setPositionFL(value);
                    motorController.setPositionBL(value);
                    Serial.print("Left motors: ");
                    Serial.println(value);
                    break;
                case 'r':
                    motorController.setPositionFR(value);
                    motorController.setPositionBR(value);
                    Serial.print("Right motors: ");
                    Serial.println(value);
                    break;
                case 'a':
                    motorController.setPositionFR(value);
                    Serial.print("FR motor: ");
                    Serial.println(value);
                    break;
                case 'b':
                    motorController.setPositionBR(value);
                    Serial.print("BR motor: ");
                    Serial.println(value);
                    break;
                case 'c':
                    motorController.setPositionFL(value);
                    Serial.print("FL motor: ");
                    Serial.println(value);
                    break;
                case 'd':
                    motorController.setPositionBL(value);
                    Serial.print("BL motor: ");
                    Serial.println(value);
                    break;
                case 'f':
                    motorController.setPositionFL(motorController.getSetpointRotationsFL() + value);
                    motorController.setPositionFR(motorController.getSetpointRotationsFR() + value);
                    motorController.setPositionBL(motorController.getSetpointRotationsBL() + value);
                    motorController.setPositionBR(motorController.getSetpointRotationsBR() + value);
                    Serial.print("Forward: ");
                    Serial.println(value);
                    break;
                case 't':
                    motorController.setPositionFL(motorController.getSetpointRotationsFL() + value);
                    motorController.setPositionBL(motorController.getSetpointRotationsBL() + value);
                    motorController.setPositionFR(motorController.getSetpointRotationsFR() - value);
                    motorController.setPositionBR(motorController.getSetpointRotationsBR() - value);
                    Serial.print("Turn: ");
                    Serial.println(value);
                    break;
                default:
                    Serial.println("Unknown motor command");
                    break;
            }
        } else if (firstChar == 'k') {
            float value = command.substring(2).toFloat();
            
            switch (secondChar) {
                case 'p':
                    motorController.setKp(value);
                    Serial.print("Kp: ");
                    Serial.println(value);
                    break;
                case 'i':
                    motorController.setKi(value);
                    Serial.print("Ki: ");
                    Serial.println(value);
                    break;
                case 'd':
                    motorController.setKd(value);
                    Serial.print("Kd: ");
                    Serial.println(value);
                    break;
                default:
                    Serial.println("Unknown PID command");
                    break;
            }
        } else if (firstChar == 'i') {
            switch (secondChar) {
                case 'r':
                    imuController.resetAngles();
                    Serial.println("[IMU] Angles reset");
                    break;
                case 'a':
                    Serial.println("\n=== IMU ANGLES ===");
                    imuController.printAngles();
                    Serial.println("==================\n");
                    break;
                case 's':
                    Serial.println("\n=== IMU STATUS ===");
                    imuController.printStatus();
                    Serial.println("==================\n");
                    break;
                case 'c': {
                    Serial.println("\n[RECALIBRATION] Keep still for 3 seconds...");
                    delay(1000);
                    
                    float gx_sum_r = 0;
                    float gy_sum_r = 0;
                    float gz_sum_r = 0;
                    int samp_r = 0;
                    
                    for (int i = 0; i < 300; i++) {
                        if (IMU.gyroscopeAvailable()) {
                            float gxr, gyr, gzr;
                            IMU.readGyroscope(gxr, gyr, gzr);
                            gx_sum_r += gxr;
                            gy_sum_r += gyr;
                            gz_sum_r += gzr;
                            samp_r++;
                        }
                        delay(10);
                    }
                    
                    if (samp_r > 0) {
                        float nx = gx_sum_r / samp_r;
                        float ny = gy_sum_r / samp_r;
                        float nz = gz_sum_r / samp_r;
                        
                        imuController.setGyroOffsets(nx, ny, nz);
                        Serial.println("[RECALIBRATION] Complete\n");
                    }
                    break;
                }
                default:
                    Serial.println("IMU: ir ia is ic");
                    break;
            }
        }
    } else if (command == "x") {
        Encoders::resetAll();
        motorController.resetPIDVariables();
        Serial.println("Motors reset");
    } else if (command == "status") {
        Serial.println("\n=== STATUS ===");
        Serial.print("FL: ");
        Serial.print(Encoders::getRotationsFL());
        Serial.print(" / ");
        Serial.println(motorController.getSetpointRotationsFL());
        Serial.print("FR: ");
        Serial.print(Encoders::getRotationsFR());
        Serial.print(" / ");
        Serial.println(motorController.getSetpointRotationsFR());
        Serial.print("BL: ");
        Serial.print(Encoders::getRotationsBL());
        Serial.print(" / ");
        Serial.println(motorController.getSetpointRotationsBL());
        Serial.print("BR: ");
        Serial.print(Encoders::getRotationsBR());
        Serial.print(" / ");
        Serial.println(motorController.getSetpointRotationsBR());
        
        Serial.print("PID: Kp=");
        Serial.print(motorController.getKp());
        Serial.print(" Ki=");
        Serial.print(motorController.getKi());
        Serial.print(" Kd=");
        Serial.println(motorController.getKd());
        
        if (imuController.isInitialized()) {
            imuController.printStatus();
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

