#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Encoders.h>

// PID Controller structure
struct PIDController {
    float kp, ki, kd;           // PID gains
    float target;               // Target value
    float current;              // Current value
    float error, lastError;     // Current and previous error
    float integral;             // Integral term
    float output;               // PID output
    float tolerance;            // Acceptable error range (stops PID when |error| < tolerance)
    unsigned long lastTime;     // Last update time
    bool enabled;               // Enable/disable PID
    bool atTarget;              // True when within tolerance
    
    PIDController() : kp(0), ki(0), kd(0), target(0), error(0), lastError(0), 
                     integral(0), output(0), tolerance(0), lastTime(0), enabled(false), atTarget(false) {}
};

// Individual Motor class
class Motor {
private:
    int pinA;
    int pinB;
    int currentSpeed;
    bool inverted;
    static Adafruit_PWMServoDriver* pwmDriver;
    static bool pwmInitialized;  // Track if PWM driver is initialized

    Encoder *encoder;
    
    
    // Control state
    enum ControlMode { MANUAL, SPEED_CONTROL, RAW_POSITION_CONTROL };
    ControlMode controlMode;
    
    void resetPID(PIDController& pid);
    
public:
    // PID Controllers
    PIDController speedPID;
    PIDController positionPID;
    // Helper methods
    float calculatePID(PIDController& pid, float currentValue, bool stopAtTarget = true);
    
    Motor(int motorPinA, int motorPinB, Encoder* enc, bool invert = false);
    
    // Basic motor control methods
    void setSpeed(int speed);           // Manual speed control (-4095 to 4095)
    void coast();                        // Stop the motor
    void brake();                       // Brake the motor (both pins high)

    // PID Speed Control
    void setTargetSpeed(float targetRPS);     // Set target speed in RPS
    void setSpeedPID(float kp, float ki, float kd);  // Configure speed PID gains
    void setSpeedTolerance(float tolerance);  // Set speed tolerance (RPS)
    void enableSpeedControl(bool enable);     // Enable/disable speed PID
    
    // PID Position Control  
    void setTargetPosition(long targetTicks);
    void setPositionPID(float kp, float ki, float kd);
    void setPositionTolerance(float tolerance);
    void enableRawPositionControl(bool enable); 
    
    // Control update (call in loop)
    void updateControl();
    
    // Configuration methods
    void setInverted(bool invert);
    bool getInverted() const { return inverted; }
    
    // Status methods
    int getCurrentSpeed() const { return currentSpeed; }
    float getTargetSpeed() const { return speedPID.target; }
    long getTargetPosition() const { return (long)positionPID.target; }
    bool isAtTarget() const; // Returns true if within tolerance
    bool isSpeedAtTarget() const { return speedPID.atTarget; }
    bool isPositionAtTarget() const { return positionPID.atTarget; }
    const char* getControlMode() const;
    Encoder* getEncoder() const { return encoder; }
    long getCurrentPosition() const { return encoder->getCount(); }
    const PIDController& getPositionPID() const { return positionPID; }

    void printStatus();
    void printPIDStatus();
    
    // Static method to initialize PWM driver (call once)
    static void initializePWM();
    static void setPWMFrequency(int frequency);
};

#endif