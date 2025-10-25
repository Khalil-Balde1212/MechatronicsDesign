#include "Encoders.h"


// Encoder class implementation
Encoder::Encoder(int encoderPinA, int encoderPinB, bool invert) 
    : pinA(encoderPinA), pinB(encoderPinB), count(0), prevCount(0), prevTime(0), inverted(invert) {
}

void Encoder::begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    reset();
    resetRPS();
}

void Encoder::reset() {
    noInterrupts();
    count = 0;
    interrupts();
}

void Encoder::resetRPS() {
    prevCount = count;
    prevTime = millis();
}

void Encoder::setInverted(bool invert) {
    inverted = invert;
}

long Encoder::getCount() {
    noInterrupts();
    long temp = count;
    interrupts();
    return temp;
}

float Encoder::getRotations() {
    return getCount() / (float)CPR;
}

float Encoder::getRPS() {
    // Sample count and time atomically, compute rate over the interval
    noInterrupts();
    unsigned long currentTime = millis();
    long currentCount = count;

    float deltaTime = (currentTime - prevTime) / 1000.0f;
    if (deltaTime <= 0.0f) {
        return 0.0f;
    }

    // Compute RPS based on change since last sample
    float rps = ((currentCount - prevCount) / (float)CPR) / deltaTime;
    prevCount = currentCount;
    prevTime = currentTime;
    interrupts();

    return rps;
}

void Encoder::updateCount() {
    bool stateA = digitalRead(pinA);
    bool stateB = digitalRead(pinB);
    
    if (stateA == stateB) {
        count += inverted ? 1 : -1;  // Invert direction if flag is set
    } else {
        count += inverted ? -1 : 1;  // Invert direction if flag is set
    }
}

void Encoder::printStatus() {
    Serial.print("Count: ");
    Serial.print(getCount());
    Serial.print(", Rotations: ");
    Serial.print(getRotations(), 2);
    Serial.print(", RPS: ");
    Serial.println(getRPS(), 2);
}
