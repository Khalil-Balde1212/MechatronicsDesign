#include "Motors.h"

// ========== PID CONTROL METHODS ==========
float Motor::calculatePID(PIDController& pid, float currentValue, bool stopAtTarget) {
    if (!pid.enabled) return 0;
    
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - pid.lastTime) / 1000.0f; // Convert ms to seconds
    
    if (deltaTime <= 0) return pid.output; // Avoid division by zero
    
    pid.error = pid.target - currentValue;

    // --- Stop at target logic ---
    if (fabs(pid.error) < pid.tolerance) {
        pid.atTarget = true;
        if (stopAtTarget) {
            pid.output = 0;
            pid.integral = 0;
            pid.lastError = pid.error;
            pid.lastTime = currentTime;
            return pid.output;
        }
    } else {
        pid.atTarget = false;
    }
    // --- End stop at target logic ---

    // Proportional term
    float P = pid.kp * pid.error;
    
    // Integral term (with windup protection)
    pid.integral += pid.error * deltaTime;
    float I = pid.ki * pid.integral;
    
    // Derivative term
    float derivative = (pid.error - pid.lastError) / deltaTime;
    float D = pid.kd * derivative;
    
    // Calculate output
    pid.output = P + I + D;
    pid.output = constrain(pid.output, -4095, 4095);
    
    // Update for next iteration
    pid.lastError = pid.error;
    pid.lastTime = currentTime;
    
    return pid.output;
}

void Motor::resetPID(PIDController& pid) {
    pid.error = 0;
    pid.lastError = 0;
    pid.integral = 0;
    pid.output = 0;
    pid.lastTime = millis();
}


// ========== CONTROL UPDATE METHOD ==========
void Motor::updateControl() {
    if (!pwmInitialized) return;
    
    int motorOutput = 0;
    
    switch (controlMode) {
        case SPEED_CONTROL:
            if (speedPID.enabled) {
                motorOutput = calculatePID(speedPID, encoder->getRPS(), false);
                motorOutput = constrain(motorOutput, -4095, 4095);
            }
            break;
            
        case RAW_POSITION_CONTROL:
            if (positionPID.enabled) {
                motorOutput = (int)(calculatePID(positionPID, encoder->getCount(), true));
                motorOutput = constrain(motorOutput, -4095, 4095);
            }
            break;
            
        case MANUAL:
            motorOutput = currentSpeed;
        default:
            return;
    }
    setSpeed(motorOutput);
}

// Status Methods
const char* Motor::getControlMode() const {
    switch (controlMode) {
        case SPEED_CONTROL: return "Speed";
        case RAW_POSITION_CONTROL: return "Position";
        case MANUAL: default: return "Manual";
    }
}

bool Motor::isAtTarget() const {
    switch (controlMode) {
        case SPEED_CONTROL: return speedPID.atTarget;
        case RAW_POSITION_CONTROL: return positionPID.atTarget;
        case MANUAL: default: return true; // Manual mode is always "at target"
    }
}

void Motor::printPIDStatus() {
    Serial.print("Motor pins ");
    Serial.print(pinA);
    Serial.print("/");
    Serial.print(pinB);
    Serial.print(" - Mode: ");
    Serial.print(getControlMode());
    
    if (controlMode == SPEED_CONTROL && speedPID.enabled) {
        Serial.print(", Target: ");
        Serial.print(speedPID.target, 2);
        Serial.print(" RPS, Error: ");
        Serial.print(speedPID.error, 2);
        Serial.print(", Output: ");
        Serial.print(speedPID.output, 0);
        Serial.print(speedPID.atTarget ? " [AT TARGET]" : "");
    } else if (controlMode == RAW_POSITION_CONTROL && positionPID.enabled) {
        Serial.print(", Target: ");
        Serial.print((long)positionPID.target);
        Serial.print(" ticks, Error: ");
        Serial.print(positionPID.error, 0);
        Serial.print(", Speed Target: ");
        Serial.print(speedPID.target, 2);
        Serial.print(positionPID.atTarget ? " [AT TARGET]" : "");
    }
    
    Serial.println();
}