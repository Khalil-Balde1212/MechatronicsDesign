#include "Motors.h"
void Motor::setSpeed(int speed) {
    if (pwmDriver == nullptr) {
        Serial.println("ERROR: PWM driver not initialized!");
        return;
    }
    if (!pwmInitialized) {
        Serial.println("ERROR: Motor::initializePWM() must be called before using setSpeed()");
        Serial.println("Motors cannot work without PWM driver initialization!");
        return;
    }
    
    // Apply inversion if set
    if (inverted) speed = -speed;
    speed = constrain(speed, -4095, 4095);
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

void Motor::coast() {
    if (pwmDriver == nullptr || !pwmInitialized) {
        Serial.println("ERROR: Cannot coast - PWM driver not initialized!");
        return;
    }
    pwmDriver->setPWM(pinA, 0, 0);
    pwmDriver->setPWM(pinB, 0, 0);
}

void Motor::brake() {
    if (pwmDriver == nullptr || !pwmInitialized) {
        Serial.println("ERROR: Cannot brake - PWM driver not initialized!");
        return;
    }
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