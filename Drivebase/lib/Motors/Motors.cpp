#include "Motors.h"

// Pin definitions


// Static PWM driver shared by all motors
Adafruit_PWMServoDriver* Motor::pwmDriver = nullptr;
bool Motor::pwmInitialized = false;  // Initialize as false

// Motor class implementation
Motor::Motor(int motorPinA, int motorPinB, bool invert) 
    : pinA(motorPinA), pinB(motorPinB), currentSpeed(0), inverted(invert) {
}

void Motor::setSpeed(int speed) {
    if (pwmDriver == nullptr) {
        Serial.println("ERROR: PWM driver not initialized!");
        return;
    }
    
    // Check if PWM driver was properly initialized with begin()
    if (!pwmInitialized) {
        Serial.println("ERROR: Motor::initializePWM() must be called before using setSpeed()");
        Serial.println("Motors cannot work without PWM driver initialization!");
        return;
    }
    
    // Apply inversion if set
    if (inverted) {
        speed = -speed;
    }
    
    // Constrain speed to valid range
    speed = constrain(speed, -4095, 4095);
    
    // Apply minimum speed threshold
    if (speed != 0 && abs(speed) < 1000) {
        speed = (speed > 0) ? 1000 : -1000;
    }
    
    currentSpeed = speed;
    
    // Set PWM values based on direction
    if (speed > 0) {
        pwmDriver->setPWM(pinA, 0, speed);
        pwmDriver->setPWM(pinB, 0, 0);
    } else if (speed < 0) {
        pwmDriver->setPWM(pinA, 0, 0);
        pwmDriver->setPWM(pinB, 0, -speed);
    } else {
        pwmDriver->setPWM(pinA, 0, 0);
        pwmDriver->setPWM(pinB, 0, 0);
    }
}

void Motor::stop() {
    setSpeed(0);
}

void Motor::brake() {
    if (pwmDriver == nullptr || !pwmInitialized) {
        Serial.println("ERROR: Cannot brake - PWM driver not initialized!");
        return;
    }
    
    // Set both pins high for braking
    pwmDriver->setPWM(pinA, 0, 4095);
    pwmDriver->setPWM(pinB, 0, 4095);
    currentSpeed = 0;
}

void Motor::setInverted(bool invert) {
    inverted = invert;
}

void Motor::printStatus() {
    Serial.print("Motor pins ");
    Serial.print(pinA);
    Serial.print("/");
    Serial.print(pinB);
    Serial.print(" - Speed: ");
    Serial.print(currentSpeed);
    Serial.print(", Inverted: ");
    Serial.println(inverted ? "Yes" : "No");
}

void Motor::initializePWM() {
    if (!pwmInitialized) {
        Serial.println("Initializing PWM driver...");
        if (pwmDriver == nullptr) {
            pwmDriver = new Adafruit_PWMServoDriver();
        }
        pwmDriver->begin();
        pwmDriver->setPWMFreq(400);
        pwmInitialized = true;
        Serial.println("PWM driver successfully initialized!");
    } else {
        Serial.println("PWM driver already initialized.");
    }
}

void Motor::setPWMFrequency(int frequency) {
    if (pwmDriver != nullptr) {
        pwmDriver->setPWMFreq(frequency);
    }
}