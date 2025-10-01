#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// Individual Motor class
class Motor {
private:
    int pinA;
    int pinB;
    int currentSpeed;
    bool inverted;
    static Adafruit_PWMServoDriver* pwmDriver;
    static bool pwmInitialized;  // Track if PWM driver is initialized
    
public:
    Motor(int motorPinA, int motorPinB, bool invert = false);
    
    // Motor control methods
    void setSpeed(int speed);           // Set motor speed (-4095 to 4095)
    void stop();                        // Stop the motor
    void brake();                       // Brake the motor (both pins high)
    
    // Configuration methods
    void setInverted(bool invert);
    bool getInverted() const { return inverted; }
    
    // Status methods
    int getCurrentSpeed() const { return currentSpeed; }
    void printStatus();
    
    // Static method to initialize PWM driver (call once)
    static void initializePWM();
    static void setPWMFrequency(int frequency);
};

#endif