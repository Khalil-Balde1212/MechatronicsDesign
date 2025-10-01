#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "../Encoders/Encoders.h"
#include "../Motors/Motors.h"

class MotorController {
public:
    MotorController();
    
    void begin();
    
    float pidControlFL();
    float pidControlFR();
    float pidControlBL();
    float pidControlBR();
    void updateAllPID();
    
    void setPositionFL(float rotations);
    void setPositionFR(float rotations);
    void setPositionBL(float rotations);
    void setPositionBR(float rotations);
    void setPositionDegreesFL(float degrees);
    void setPositionDegreesFR(float degrees);
    void setPositionDegreesBL(float degrees);
    void setPositionDegreesBR(float degrees);
    
    void setKp(float kp_new) { kp = kp_new; }
    void setKi(float ki_new) { ki = ki_new; }
    void setKd(float kd_new) { kd = kd_new; }
    
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
    
    long getSetpointFL() const { return setpoint_fl; }
    long getSetpointFR() const { return setpoint_fr; }
    long getSetpointBL() const { return setpoint_bl; }
    long getSetpointBR() const { return setpoint_br; }
    
    float getSetpointRotationsFL() const { return setpoint_fl / static_cast<float>(cpr); }
    float getSetpointRotationsFR() const { return setpoint_fr / static_cast<float>(cpr); }
    float getSetpointRotationsBL() const { return setpoint_bl / static_cast<float>(cpr); }
    float getSetpointRotationsBR() const { return setpoint_br / static_cast<float>(cpr); }
    
    void resetPIDVariables();
    
    // Speed PID control methods
    float speedPidFL();
    float speedPidFR();
    float speedPidBL();
    float speedPidBR();
    void updateAllSpeedPID();
    
    // Speed PID setters and getters
    void setSpeedKp(float kp_new) { speed_kp = kp_new; }
    void setSpeedKi(float ki_new) { speed_ki = ki_new; }
    void setSpeedKd(float kd_new) { speed_kd = kd_new; }
    
    float getSpeedKp() const { return speed_kp; }
    float getSpeedKi() const { return speed_ki; }
    float getSpeedKd() const { return speed_kd; }
    
    // Speed setpoint setters (in RPM)
    void setSpeedSetpointFL(float rpm) { speed_setpoint_fl = rpm; }
    void setSpeedSetpointFR(float rpm) { speed_setpoint_fr = rpm; }
    void setSpeedSetpointBL(float rpm) { speed_setpoint_bl = rpm; }
    void setSpeedSetpointBR(float rpm) { speed_setpoint_br = rpm; }
    
    // Speed setpoint getters
    float getSpeedSetpointFL() const { return speed_setpoint_fl; }
    float getSpeedSetpointFR() const { return speed_setpoint_fr; }
    float getSpeedSetpointBL() const { return speed_setpoint_bl; }
    float getSpeedSetpointBR() const { return speed_setpoint_br; }
    
    // Mode toggle
    void setSpeedMode(bool mode) { speed_mode = mode; }
    bool getSpeedMode() const { return speed_mode; }
    
private:
    static constexpr int cpr = 1440; // Counts per revolution
    static constexpr long HOLD_DEADBAND_COUNTS = 8;      // encoder tolerance band
    static constexpr float INTEGRAL_FREEZE_COUNTS = 4.0f; // freeze I-term near zero
    static constexpr int PID_MIN_EFFORT = 700;           // minimum PWM magnitude
    static constexpr float DERIVATIVE_ALPHA = 0.6f;      // low-pass filter on derivative

    float kp = 25.0f;
    float ki = 0.0f;
    float kd = 0.9f;
    
    long setpoint_fl = 0, setpoint_fr = 0, setpoint_bl = 0, setpoint_br = 0;
    float integral_fl = 0.0f, integral_fr = 0.0f, integral_bl = 0.0f, integral_br = 0.0f;
    float prev_error_fl = 0.0f, prev_error_fr = 0.0f, prev_error_bl = 0.0f, prev_error_br = 0.0f;
    float filtered_derivative_fl = 0.0f, filtered_derivative_fr = 0.0f, filtered_derivative_bl = 0.0f, filtered_derivative_br = 0.0f;
    unsigned long last_time_fl = 0, last_time_fr = 0, last_time_bl = 0, last_time_br = 0;
    
    // Speed PID variables
    float speed_kp = 2.0f;
    float speed_ki = 0.1f;
    float speed_kd = 0.05f;
    
    float speed_setpoint_fl = 0.0f, speed_setpoint_fr = 0.0f, speed_setpoint_bl = 0.0f, speed_setpoint_br = 0.0f;
    float speed_integral_fl = 0.0f, speed_integral_fr = 0.0f, speed_integral_bl = 0.0f, speed_integral_br = 0.0f;
    float speed_prev_error_fl = 0.0f, speed_prev_error_fr = 0.0f, speed_prev_error_bl = 0.0f, speed_prev_error_br = 0.0f;
    unsigned long speed_last_time_fl = 0, speed_last_time_fr = 0, speed_last_time_bl = 0, speed_last_time_br = 0;
    long speed_prev_count_fl = 0, speed_prev_count_fr = 0, speed_prev_count_bl = 0, speed_prev_count_br = 0;
    
    // Mode toggle: false = position PID, true = speed PID
    bool speed_mode = false;
};

#endif
