#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

// Individual Encoder class
class Encoder {
private:
    int pinA;
    int pinB;
    volatile long count;
    long prevCount;
    unsigned long prevTime;
    bool inverted;                   // Flag to invert encoder direction
    float maxRPM = 65.6; // Measured at 5v
    
    static const int CPR = 2940; //7 * 210:1 per 2x quadrature

public:
    Encoder(int encoderPinA, int encoderPinB, bool invert = false);
    void begin();
    // Initialize pins and setup
    void reset();                    // Reset count to zero
    void resetRPS();                 // Reset RPS calculation baseline
    
    // Invert control methods
    void setInverted(bool invert);
    bool getInverted() const { return inverted; }
    
    // Data access methods
    long getCount();
    float getRotations();
    float getRPS();
    
    // ISR method (called by interrupt)
    void updateCount();
    
    // Utility methods
    void printStatus();
    int getPinA() const { return pinA; }
    int getPinB() const { return pinB; }
};

#endif