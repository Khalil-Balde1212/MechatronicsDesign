#include <Arduino.h>
#include "RobotMap.h"
#include "Encoders.h"
#include "Motors.h"

// Global encoder objects
Encoder encoderFL(RobotMap::ENC_FLA, RobotMap::ENC_FLB);
Encoder encoderFR(RobotMap::ENC_FRA, RobotMap::ENC_FRB);
Encoder encoderBL(RobotMap::ENC_BLA, RobotMap::ENC_BLB);
Encoder encoderBR(RobotMap::ENC_BRA, RobotMap::ENC_BRB);

Motor motorFL(RobotMap::MOTOR_FLA, RobotMap::MOTOR_FLB);
Motor motorFR(RobotMap::MOTOR_FRA, RobotMap::MOTOR_FRB);
Motor motorBL(RobotMap::MOTOR_BLA, RobotMap::MOTOR_BLB);
Motor motorBR(RobotMap::MOTOR_BRA, RobotMap::MOTOR_BRB);

void setup() {
    Serial.begin(9600);
    
    // Initialize motor PWM driver
    Motor::initializePWM();
    Serial.println("Motor PWM initialized");
    
    // Initialize all encoders
    encoderFL.begin();
    encoderFR.begin();
    encoderBL.begin();
    encoderBR.begin();

    // encoderBL.setInverted(true); // Invert left back encoder if needed
    // motorBR.setInverted(true);   // Invert left back motor if needed
    
    // Attach interrupts for all encoders
    attachInterrupt(digitalPinToInterrupt(encoderFL.getPinA()), []() { encoderFL.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderFR.getPinA()), []() { encoderFR.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBL.getPinA()), []() { encoderBL.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBR.getPinA()), []() { encoderBR.updateCount(); }, CHANGE);
    
    motorFL.stop();
    motorFR.stop();
    motorBL.stop();
    motorBR.stop();
    
    Serial.println("Setup complete - Motor and Encoders ready!");
    
    // Configure PID gains for back right motor only
    motorBR.setSpeedPID(1000, 100, 50);
    motorBR.setPositionPID(5.0, 0.1, 1.0);
    
    // Set tolerances - PID will stop when within these errors
    motorBR.setSpeedTolerance(0.1);      // Stop when within 0.1 RPS
    motorBR.setPositionTolerance(20.0);   // Stop when within 20 ticks (~1.4% of rotation)
    
    // Enable position control for back right motor
    motorBR.enablePositionControl(true);
    
    Serial.println("=== BACK RIGHT MOTOR CONTROL ===");
    Serial.println("PID stops when within tolerance of target");
    Serial.println("Commands:");
    Serial.println("  Position Control:");
    Serial.println("    1440      → Move to position 1440 ticks (+1 rotation)");
    Serial.println("    -720      → Move to position -720 ticks (-0.5 rotation)");
    Serial.println("    0         → Move to position 0 (home)");
    Serial.println("  Speed Control:");
    Serial.println("    s2.5      → Set speed to 2.5 RPS");
    Serial.println("    s-1.0     → Set speed to -1.0 RPS (reverse)");
    Serial.println("    s0        → Set speed to 0 RPS (stop)");
    Serial.println("  Utility:");
    Serial.println("    reset     → Reset encoder to zero");
    Serial.println("    stop      → Disable all control (manual mode)");
    Serial.print("Current position: ");
    Serial.println(encoderBR.getCount());
    Serial.println();
}

void loop() {
    // Update back right motor control loop
    motorBR.updateControl(encoderBR.getRPS(), encoderBR.getCount());
    
    // Check for serial input
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove whitespace
        
        if (input.equalsIgnoreCase("reset")) {
            encoderBR.reset();
            Serial.println("Encoder reset to 0");
            
        } else if (input.equalsIgnoreCase("stop")) {
            motorBR.enablePositionControl(false);
            motorBR.enableSpeedControl(false);
            motorBR.stop();
            Serial.println("All control disabled - motor stopped (manual mode)");
            
        } else if (input.startsWith("s") || input.startsWith("S")) {
            // Speed control command (e.g., "s2.5", "s-1.0", "s0")
            String speedStr = input.substring(1); // Remove 's' prefix
            float targetSpeed = speedStr.toFloat();
            
            if (speedStr.equals("0") || targetSpeed != 0.0 || speedStr.indexOf("0") >= 0) {
                motorBR.enableSpeedControl(true);
                motorBR.setTargetSpeed(targetSpeed);
                Serial.print("Speed control enabled - Target: ");
                Serial.print(targetSpeed, 1);
                Serial.println(" RPS");
            } else {
                Serial.println("Invalid speed command. Use format: s2.5, s-1.0, s0");
            }
            
        } else {
            // Try to parse as position number
            long newPosition = input.toInt();
            if (input.equals("0") || newPosition != 0) {
                motorBR.enablePositionControl(true);
                motorBR.setTargetPosition(newPosition);
                Serial.print("Position control enabled - Target: ");
                Serial.print(newPosition);
                Serial.println(" ticks");
            } else {
                Serial.println("Invalid command. Examples:");
                Serial.println("  Position: 1440, -720, 0");
                Serial.println("  Speed: s2.5, s-1.0, s0");
                Serial.println("  Utility: reset, stop");
            }
        }
    }

    // Print status every second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
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
        if (strcmp(motorBR.getControlMode(), "Position") == 0) {
            Serial.print(", Pos Target: ");
            Serial.print(motorBR.getTargetPosition());
            Serial.print(", Error: ");
            Serial.print(motorBR.getTargetPosition() - encoderBR.getCount());
        } else if (strcmp(motorBR.getControlMode(), "Speed") == 0) {
            Serial.print(", Speed Target: ");
            Serial.print(motorBR.getTargetSpeed(), 1);
            Serial.print(" RPS, Error: ");
            Serial.print(motorBR.getTargetSpeed() - encoderBR.getRPS(), 2);
        }
        
        // Show if motor has reached target
        if (motorBR.isAtTarget()) {
            Serial.print(" ✓ AT TARGET");
        } else {
            Serial.print(" → Moving");
        }
        
        Serial.println();
    }
}