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

    encoderBL.setInverted(true); // Invert left back encoder if needed
    motorBR.setInverted(true);   // Invert left back motor if needed
    
    // Attach interrupts for all encoders
    attachInterrupt(digitalPinToInterrupt(encoderFL.getPinA()), []() { encoderFL.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderFR.getPinA()), []() { encoderFR.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBL.getPinA()), []() { encoderBL.updateCount(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBR.getPinA()), []() { encoderBR.updateCount(); }, CHANGE);
    
    motorFL.stop();
    motorFR.stop();
    motorBL.stop();
    motorBR.stop();
}

void loop() {
    // Simple motor test sequence every 5 seconds
   

    // Print encoder status every second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        encoderFL.printStatus();
        encoderFR.printStatus();
        encoderBL.printStatus();
        encoderBR.printStatus();
        Serial.println("-----");
    }
}