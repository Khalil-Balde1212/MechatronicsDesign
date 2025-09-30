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
    
    // Setpoint getters in rotations  
    float getSetpointRotationsFL() const { return setpoint_fl / (float)cpr; }
    float getSetpointRotationsFR() const { return setpoint_fr / (float)cpr; }
    float getSetpointRotationsBL() const { return setpoint_bl / (float)cpr; }
    float getSetpointRotationsBR() const { return setpoint_br / (float)cpr; }
    
    void resetPIDVariables();
    
private:
    static const int cpr = 1440; // Counts per revolution
    
    float kp = 25.0;
    float ki = 0.0;
    float kd = 0.9;
    
    long setpoint_fl = 0, setpoint_fr = 0, setpoint_bl = 0, setpoint_br = 0;
    float integral_fl = 0, integral_fr = 0, integral_bl = 0, integral_br = 0;
    float prev_error_fl = 0, prev_error_fr = 0, prev_error_bl = 0, prev_error_br = 0;
    unsigned long last_time_fl = 0, last_time_fr = 0, last_time_bl = 0, last_time_br = 0;
};

#endif