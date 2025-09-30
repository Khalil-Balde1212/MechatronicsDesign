#include "MotorController.h"

MotorController::MotorController() {
    setpoint_fl = setpoint_fr = setpoint_bl = setpoint_br = 0;
    integral_fl = integral_fr = integral_bl = integral_br = 0;
    prev_error_fl = prev_error_fr = prev_error_bl = prev_error_br = 0;
    last_time_fl = last_time_fr = last_time_bl = last_time_br = 0;
}

void MotorController::begin() {
    Motors::initialize();
    Encoders::initializePins();
    Encoders::attachInterrupts();
    
    Motors::stopAll();
    last_time_fl = last_time_fr = last_time_bl = last_time_br = millis();
    
    Serial.println("Motor Controller Initialized");
    Serial.println("Commands:");
    Serial.println("- rX####: Rotate motors (X: side, ####: rotations)");
    Serial.println("  Sides: r=right, l=left, a=front right, b=back right, c=front left, d=back left");
    Serial.println("  f=forward all, t=turn all");
    Serial.println("- kX####: Set PID gains (X: p=kp, i=ki, d=kd, ####: value)");
    Serial.println("- x: Reset all positions");
    Serial.println("- status: Display all current positions and setpoints");
}

float MotorController::pidControlFL() {
    long current_pos = Encoders::getCountFL();
    long error = setpoint_fl - current_pos;
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time_fl) / 1000.0;
    
    if (delta_time > 0) {
        integral_fl += error * delta_time;
        integral_fl = constrain(integral_fl, -100, 100);
        
        float derivative = (error - prev_error_fl) / delta_time;
        float output = kp * error + ki * integral_fl + kd * derivative;
        output = constrain(output, -4095, 4095);
        
        prev_error_fl = error;
        last_time_fl = current_time;
        
        Motors::setSpeedFL(output);
        return output;
    }
    return 0;
}

float MotorController::pidControlFR() {
    long current_pos = Encoders::getCountFR();
    long error = setpoint_fr - current_pos;
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time_fr) / 1000.0;
    
    if (delta_time > 0) {
        integral_fr += error * delta_time;
        integral_fr = constrain(integral_fr, -100, 100);
        
        float derivative = (error - prev_error_fr) / delta_time;
        float output = kp * error + ki * integral_fr + kd * derivative;
        output = constrain(output, -4095, 4095);
        
        prev_error_fr = error;
        last_time_fr = current_time;
        
        Motors::setSpeedFR(output);
        return output;
    }
    return 0;
}

float MotorController::pidControlBL() {
    long current_pos = Encoders::getCountBL();
    long error = setpoint_bl - current_pos;
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time_bl) / 1000.0;
    
    if (delta_time > 0) {
        integral_bl += error * delta_time;
        integral_bl = constrain(integral_bl, -100, 100);
        
        float derivative = (error - prev_error_bl) / delta_time;
        float output = kp * error + ki * integral_bl + kd * derivative;
        output = constrain(output, -4095, 4095);
        
        prev_error_bl = error;
        last_time_bl = current_time;
        
        Motors::setSpeedBL(output);
        return output;
    }
    return 0;
}

float MotorController::pidControlBR() {
    long current_pos = Encoders::getCountBR();
    long error = setpoint_br - current_pos;
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time_br) / 1000.0;
    
    if (delta_time > 0) {
        integral_br += error * delta_time;
        integral_br = constrain(integral_br, -100, 100);
        
        float derivative = (error - prev_error_br) / delta_time;
        float output = kp * error + ki * integral_br + kd * derivative;
        output = constrain(output, -4095, 4095);
        
        prev_error_br = error;
        last_time_br = current_time;
        
        Motors::setSpeedBR(output);
        return output;
    }
    return 0;
}

void MotorController::updateAllPID() {
    pidControlFL();
    pidControlFR();
    pidControlBL();
    pidControlBR();
}

void MotorController::setPositionFL(float rotations) {
    setpoint_fl = rotations * cpr;
    integral_fl = 0;
    prev_error_fl = 0;
    last_time_fl = millis();
}

void MotorController::setPositionFR(float rotations) {
    setpoint_fr = rotations * cpr;
    integral_fr = 0;
    prev_error_fr = 0;
    last_time_fr = millis();
}

void MotorController::setPositionBL(float rotations) {
    setpoint_bl = rotations * cpr;
    integral_bl = 0;
    prev_error_bl = 0;
    last_time_bl = millis();
}

void MotorController::setPositionBR(float rotations) {
    setpoint_br = rotations * cpr;
    integral_br = 0;
    prev_error_br = 0;
    last_time_br = millis();
}

void MotorController::setPositionDegreesFL(float degrees) {
    setPositionFL(degrees / 360.0);
}

void MotorController::setPositionDegreesFR(float degrees) {
    setPositionFR(degrees / 360.0);
}

void MotorController::setPositionDegreesBL(float degrees) {
    setPositionBL(degrees / 360.0);
}

void MotorController::setPositionDegreesBR(float degrees) {
    setPositionBR(degrees / 360.0);
}

// Reset PID variables
void MotorController::resetPIDVariables() {
    setpoint_fl = setpoint_fr = setpoint_bl = setpoint_br = 0;
    integral_fl = integral_fr = integral_bl = integral_br = 0;
    prev_error_fl = prev_error_fr = prev_error_bl = prev_error_br = 0;
}