#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

namespace Encoders {
    const int ENC_FLA = 10;
    const int ENC_FLB = 9;
    const int ENC_FRA = 11;
    const int ENC_FRB = 8;
    const int ENC_BLA = 3;
    const int ENC_BLB = 4;
    const int ENC_BRA = 2;
    const int ENC_BRB = 7;

    const int CPR = 1440; 
    
    extern volatile long countFL;
    extern volatile long countFR;
    extern volatile long countBL;
    extern volatile long countBR;
    
    void initializePins();
    void attachInterrupts();
    
    void resetCounts();
    void resetAll();
    void printCounts();
    void printRotations();
    void printRPS();
    
    long getCountFL();
    long getCountFR();
    long getCountBL();
    long getCountBR();
    
    float getRotationsFL();
    float getRotationsFR();
    float getRotationsBL();
    float getRotationsBR();
    
    float getRPSFL();
    float getRPSFR();
    float getRPSBL();
    float getRPSBR();
    
    void ISR_FL();
    void ISR_FR();
    void ISR_BL();
    void ISR_BR();
}

// ISR Wrappers
void encoderISR_FL();
void encoderISR_FR();
void encoderISR_BL();
void encoderISR_BR();

#endif