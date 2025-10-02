#include "Motors.h"

// Pin definitions


// Static PWM driver shared by all motors
Adafruit_PWMServoDriver* Motor::pwmDriver = nullptr;
bool Motor::pwmInitialized = false;  // Initialize as false

// Motor class implementation
Motor::Motor(int motorPinA, int motorPinB, bool invert) 
    : pinA(motorPinA), pinB(motorPinB), currentSpeed(0), inverted(invert), controlMode(MANUAL) {
    // Initialize PID controllers with default gains
    speedPID.kp = 10.0;   // Default speed PID gains
    speedPID.ki = 0.0;
    speedPID.kd = 0.0;
    speedPID.tolerance = 0.1; // Default speed tolerance: 0.1 RPS
    
    positionPID.kp = 25.0;   // Default position PID gains  
    positionPID.ki = 0.0;
    positionPID.kd = 0.9;
    positionPID.tolerance = 10.0; // Default position tolerance: 10 ticks
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
        pwmDriver->setPWM(pinA, 0, 4096);
        pwmDriver->setPWM(pinB, 0, 4096);
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

// ========== PID CONTROL METHODS ==========

float Motor::calculatePID(PIDController& pid, float currentValue, bool stopAtTarget) {
    if (!pid.enabled) return 0;
    
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - pid.lastTime) / 1000.0; // Convert to seconds
    
    if (deltaTime <= 0) return pid.output; // Avoid division by zero
    
    // Calculate error
    pid.error = pid.target - currentValue;
    
    // Check if within tolerance
    if (abs(pid.error) <= pid.tolerance) {
        pid.atTarget = true;
        
        // For speed control, continue regulating even when at target
        // For position control, stop when at target
        if (stopAtTarget) {
            pid.output = 0; // Stop PID output when within tolerance
            pid.integral = 0; // Reset integral to prevent windup
            pid.lastError = pid.error;
            pid.lastTime = currentTime;
            return 0;
        }
        // If not stopping at target, continue with normal PID calculation below
    } else {
        pid.atTarget = false;
    }
    
    // Proportional term
    float P = pid.kp * pid.error;
    
    // Integral term (with windup protection)
    pid.integral += pid.error * deltaTime;
    pid.integral = constrain(pid.integral, -4095/max(pid.ki, 0.01f), 4095/max(pid.ki, 0.01f)); // Prevent windup
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

// Speed Control Methods
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

// Position Control Methods
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

void Motor::enablePositionControl(bool enable) {
    positionPID.enabled = enable;
    if (enable) {
        controlMode = POSITION_CONTROL;
        resetPID(positionPID);
        // Disable speed control
        speedPID.enabled = false;
    } else if (controlMode == POSITION_CONTROL) {
        controlMode = MANUAL;
    }
}

// Main control update method
void Motor::updateControl(float currentRPS, long currentPosition) {
    if (!pwmInitialized) return;
    
    int motorOutput = 0;
    
    switch (controlMode) {
        case SPEED_CONTROL:
            if (speedPID.enabled) {
                // Speed control: PID output directly sets motor PWM
                // Convert target RPS to base PWM (rough approximation: 1 RPS ≈ 800 PWM)
                float basePWM = speedPID.target * 800.0; // Base PWM for target speed
                float pidCorrection = calculatePID(speedPID, currentRPS, false);
                motorOutput = (int)(basePWM + pidCorrection);
                motorOutput = constrain(motorOutput, -4095, 4095);
            }
            break;
            
        case POSITION_CONTROL:
            if (positionPID.enabled) {
                // Position control outputs desired speed, then speed control handles motor output
                float targetSpeed = calculatePID(positionPID, (float)currentPosition, true); // Position stops at target
                targetSpeed = constrain(targetSpeed, -20.0, 20.0); // Limit max speed to 20 RPS
                
                // Use position output as speed target for inner speed loop
                speedPID.target = targetSpeed;
                speedPID.enabled = true;
                // Inner speed loop for position control: convert speed to PWM
                float basePWM = speedPID.target * 800.0;
                float pidCorrection = calculatePID(speedPID, currentRPS, false);
                motorOutput = (int)(basePWM + pidCorrection);
                motorOutput = constrain(motorOutput, -4095, 4095);
            }
            break;
            
        case MANUAL:
        default:
            // Manual control - do nothing, setSpeed() handles this
            return;
    }
    
    // Apply motor output
    setSpeed(motorOutput);
}

// Status Methods
const char* Motor::getControlMode() const {
    switch (controlMode) {
        case SPEED_CONTROL: return "Speed";
        case POSITION_CONTROL: return "Position";
        case MANUAL: default: return "Manual";
    }
}

bool Motor::isAtTarget() const {
    switch (controlMode) {
        case SPEED_CONTROL: return speedPID.atTarget;
        case POSITION_CONTROL: return positionPID.atTarget;
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
    } else if (controlMode == POSITION_CONTROL && positionPID.enabled) {
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