#include "MotorController.h"
#include <math.h>

MotorController::MotorController() {
    setpoint_fl = setpoint_fr = setpoint_bl = setpoint_br = 0;
    integral_fl = integral_fr = integral_bl = integral_br = 0.0f;
    prev_error_fl = prev_error_fr = prev_error_bl = prev_error_br = 0.0f;
    filtered_derivative_fl = filtered_derivative_fr = filtered_derivative_bl = filtered_derivative_br = 0.0f;
    last_time_fl = last_time_fr = last_time_bl = last_time_br = 0;
}

void MotorController::begin() {
    Motors::initialize();
    Encoders::initializePins();
    Encoders::attachInterrupts();
    Encoders::resetAll();

    Motors::stopAll();
    unsigned long now = millis();
    last_time_fl = last_time_fr = last_time_bl = last_time_br = now;

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
    float delta_time = (current_time - last_time_fl) / 1000.0f;

    if (delta_time <= 0.0f) {
        return 0.0f;
    }

    if (labs(error) <= HOLD_DEADBAND_COUNTS) {
        integral_fl = 0.0f;
        filtered_derivative_fl = 0.0f;
        prev_error_fl = static_cast<float>(error);
        last_time_fl = current_time;
        Motors::setSpeedFL(0);
        return 0.0f;
    }

    if (labs(error) > INTEGRAL_FREEZE_COUNTS) {
        integral_fl += error * delta_time;
        integral_fl = constrain(integral_fl, -100.0f, 100.0f);
    }

    float derivative = (error - prev_error_fl) / delta_time;
    filtered_derivative_fl = (DERIVATIVE_ALPHA * filtered_derivative_fl) + ((1.0f - DERIVATIVE_ALPHA) * derivative);

    float output = kp * error + ki * integral_fl + kd * filtered_derivative_fl;
    output = constrain(output, -4095.0f, 4095.0f);

    prev_error_fl = static_cast<float>(error);
    last_time_fl = current_time;

    if (fabsf(output) < PID_MIN_EFFORT) {
        Motors::setSpeedFL(0);
        return 0.0f;
    }

    Motors::setSpeedFL(static_cast<int>(output));
    return output;
}

float MotorController::pidControlFR() {
    long current_pos = Encoders::getCountFR();
    long error = setpoint_fr - current_pos;
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time_fr) / 1000.0f;

    if (delta_time <= 0.0f) {
        return 0.0f;
    }

    if (labs(error) <= HOLD_DEADBAND_COUNTS) {
        integral_fr = 0.0f;
        filtered_derivative_fr = 0.0f;
        prev_error_fr = static_cast<float>(error);
        last_time_fr = current_time;
        Motors::setSpeedFR(0);
        return 0.0f;
    }

    if (labs(error) > INTEGRAL_FREEZE_COUNTS) {
        integral_fr += error * delta_time;
        integral_fr = constrain(integral_fr, -100.0f, 100.0f);
    }

    float derivative = (error - prev_error_fr) / delta_time;
    filtered_derivative_fr = (DERIVATIVE_ALPHA * filtered_derivative_fr) + ((1.0f - DERIVATIVE_ALPHA) * derivative);

    float output = kp * error + ki * integral_fr + kd * filtered_derivative_fr;
    output = constrain(output, -4095.0f, 4095.0f);

    prev_error_fr = static_cast<float>(error);
    last_time_fr = current_time;

    if (fabsf(output) < PID_MIN_EFFORT) {
        Motors::setSpeedFR(0);
        return 0.0f;
    }

    Motors::setSpeedFR(static_cast<int>(output));
    return output;
}

float MotorController::pidControlBL() {
    long current_pos = Encoders::getCountBL();
    long error = setpoint_bl - current_pos;
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time_bl) / 1000.0f;

    if (delta_time <= 0.0f) {
        return 0.0f;
    }

    if (labs(error) <= HOLD_DEADBAND_COUNTS) {
        integral_bl = 0.0f;
        filtered_derivative_bl = 0.0f;
        prev_error_bl = static_cast<float>(error);
        last_time_bl = current_time;
        Motors::setSpeedBL(0);
        return 0.0f;
    }

    if (labs(error) > INTEGRAL_FREEZE_COUNTS) {
        integral_bl += error * delta_time;
        integral_bl = constrain(integral_bl, -100.0f, 100.0f);
    }

    float derivative = (error - prev_error_bl) / delta_time;
    filtered_derivative_bl = (DERIVATIVE_ALPHA * filtered_derivative_bl) + ((1.0f - DERIVATIVE_ALPHA) * derivative);

    float output = kp * error + ki * integral_bl + kd * filtered_derivative_bl;
    output = constrain(output, -4095.0f, 4095.0f);

    prev_error_bl = static_cast<float>(error);
    last_time_bl = current_time;

    if (fabsf(output) < PID_MIN_EFFORT) {
        Motors::setSpeedBL(0);
        return 0.0f;
    }

    Motors::setSpeedBL(static_cast<int>(output));
    return output;
}

float MotorController::pidControlBR() {
    long current_pos = Encoders::getCountBR();
    long error = setpoint_br - current_pos;
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time_br) / 1000.0f;

    if (delta_time <= 0.0f) {
        return 0.0f;
    }

    if (labs(error) <= HOLD_DEADBAND_COUNTS) {
        integral_br = 0.0f;
        filtered_derivative_br = 0.0f;
        prev_error_br = static_cast<float>(error);
        last_time_br = current_time;
        Motors::setSpeedBR(0);
        return 0.0f;
    }

    if (labs(error) > INTEGRAL_FREEZE_COUNTS) {
        integral_br += error * delta_time;
        integral_br = constrain(integral_br, -100.0f, 100.0f);
    }

    float derivative = (error - prev_error_br) / delta_time;
    filtered_derivative_br = (DERIVATIVE_ALPHA * filtered_derivative_br) + ((1.0f - DERIVATIVE_ALPHA) * derivative);

    float output = kp * error + ki * integral_br + kd * filtered_derivative_br;
    output = constrain(output, -4095.0f, 4095.0f);

    prev_error_br = static_cast<float>(error);
    last_time_br = current_time;

    if (fabsf(output) < PID_MIN_EFFORT) {
        Motors::setSpeedBR(0);
        return 0.0f;
    }

    Motors::setSpeedBR(static_cast<int>(output));
    return output;
}

void MotorController::updateAllPID() {
    pidControlFL();
    pidControlFR();
    pidControlBL();
    pidControlBR();
}

void MotorController::setPositionFL(float rotations) {
    setpoint_fl = rotations * cpr;
    integral_fl = 0.0f;
    prev_error_fl = 0.0f;
    filtered_derivative_fl = 0.0f;
    last_time_fl = millis();
}

void MotorController::setPositionFR(float rotations) {
    setpoint_fr = rotations * cpr;
    integral_fr = 0.0f;
    prev_error_fr = 0.0f;
    filtered_derivative_fr = 0.0f;
    last_time_fr = millis();
}

void MotorController::setPositionBL(float rotations) {
    setpoint_bl = rotations * cpr;
    integral_bl = 0.0f;
    prev_error_bl = 0.0f;
    filtered_derivative_bl = 0.0f;
    last_time_bl = millis();
}

void MotorController::setPositionBR(float rotations) {
    setpoint_br = rotations * cpr;
    integral_br = 0.0f;
    prev_error_br = 0.0f;
    filtered_derivative_br = 0.0f;
    last_time_br = millis();
}

void MotorController::setPositionDegreesFL(float degrees) {
    setPositionFL(degrees / 360.0f);
}

void MotorController::setPositionDegreesFR(float degrees) {
    setPositionFR(degrees / 360.0f);
}

void MotorController::setPositionDegreesBL(float degrees) {
    setPositionBL(degrees / 360.0f);
}

void MotorController::setPositionDegreesBR(float degrees) {
    setPositionBR(degrees / 360.0f);
}

void MotorController::resetPIDVariables() {
    setpoint_fl = setpoint_fr = setpoint_bl = setpoint_br = 0;
    integral_fl = integral_fr = integral_bl = integral_br = 0.0f;
    prev_error_fl = prev_error_fr = prev_error_bl = prev_error_br = 0.0f;
    filtered_derivative_fl = filtered_derivative_fr = filtered_derivative_bl = filtered_derivative_br = 0.0f;
}



