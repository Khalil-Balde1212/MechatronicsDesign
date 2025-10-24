#include "Motors.h"

// Static PWM driver shared by all motors
Adafruit_PWMServoDriver* Motor::pwmDriver = nullptr;
bool Motor::pwmInitialized = false;  // Initialize as false

Motor::Motor(int motorPinA, int motorPinB, Encoder* enc, bool invert) 
    : pinA(motorPinA), pinB(motorPinB), currentSpeed(0), inverted(invert), controlMode(MANUAL) {
    speedPID.kp = 10.0;   // Default speed PID gains
    speedPID.ki = 0.0;
    speedPID.kd = 0.0;
    speedPID.tolerance = 0.1;
    
    positionPID.kp = 0.0; 
    positionPID.ki = 0.0;
    positionPID.kd = 0.0;
    positionPID.tolerance = 10.0;


    encoder = enc;
}




// ========== SPEED CONTROL METHODS ==========
void Motor::setTargetSpeed(float targetRPS) {
    speedPID.target = targetRPS;
    resetPID(speedPID);
}

void Motor::setSpeedPID(float kp, float ki, float kd) {
    speedPID.kp = kp;
    speedPID.ki = ki;
    speedPID.kd = kd;
    resetPID(speedPID);
}

void Motor::setSpeedTolerance(float tolerance) {
    speedPID.tolerance = tolerance;
}

void Motor::enableSpeedControl(bool enable) {
    speedPID.enabled = enable;
    if (enable) {
        controlMode = SPEED_CONTROL;
        resetPID(speedPID);
        // Disable position control
        positionPID.enabled = false;
    } else if (controlMode == SPEED_CONTROL) {
        controlMode = MANUAL;
    }
}

// ========== POSITION CONTROL METHODS ==========
void Motor::setTargetPosition(long targetTicks) {
    positionPID.target = (float)targetTicks;
    resetPID(positionPID);
}

void Motor::setPositionPID(float kp, float ki, float kd) {
    positionPID.kp = kp;
    positionPID.ki = ki;
    positionPID.kd = kd;
    resetPID(positionPID);
}

void Motor::setPositionTolerance(float tolerance) {
    positionPID.tolerance = tolerance;
}

void Motor::enableRawPositionControl(bool enable) {
    positionPID.enabled = enable;
    if (enable) {
        controlMode = RAW_POSITION_CONTROL;
        resetPID(positionPID);
        speedPID.enabled = false;
    } else if (controlMode == RAW_POSITION_CONTROL) {
        controlMode = MANUAL;
    }
}

