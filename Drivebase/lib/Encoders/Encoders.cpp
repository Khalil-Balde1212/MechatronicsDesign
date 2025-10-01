#include "Encoders.h"

namespace Encoders {
    volatile long countFL = 0;
    volatile long countFR = 0;
    volatile long countBL = 0;
    volatile long countBR = 0;
    
    void initializePins() {
        pinMode(ENC_FLA, INPUT_PULLUP);
        pinMode(ENC_FLB, INPUT_PULLUP);
        pinMode(ENC_FRA, INPUT_PULLUP);
        pinMode(ENC_FRB, INPUT_PULLUP);
        pinMode(ENC_BLA, INPUT_PULLUP);
        pinMode(ENC_BLB, INPUT_PULLUP);
        pinMode(ENC_BRA, INPUT_PULLUP);
        pinMode(ENC_BRB, INPUT_PULLUP);
    }

    void attachInterrupts() {
        Serial.println("Encoder interrupt pins:");
        Serial.println(digitalPinToInterrupt(ENC_FLA));
        Serial.println(digitalPinToInterrupt(ENC_FRA));
        Serial.println(digitalPinToInterrupt(ENC_BLA));
        Serial.println(digitalPinToInterrupt(ENC_BRA));
        
        attachInterrupt(digitalPinToInterrupt(ENC_FLA), ::encoderISR_FL, RISING);
        attachInterrupt(digitalPinToInterrupt(ENC_FRA), ::encoderISR_FR, RISING);
        attachInterrupt(digitalPinToInterrupt(ENC_BLA), ::encoderISR_BL, RISING);
        attachInterrupt(digitalPinToInterrupt(ENC_BRA), ::encoderISR_BR, RISING);
    }
    
    void resetCounts() {
        noInterrupts();
        countFL = 0;
        countFR = 0;
        countBL = 0;
        countBR = 0;
        interrupts();
    }
    
    void resetAll() {
        resetCounts();
    }
    
    void printCounts() {
        noInterrupts();
        long tempFL = countFL;
        long tempFR = countFR;
        long tempBL = countBL;
        long tempBR = countBR;
        interrupts();
        
        Serial.print("Encoders - FL: ");
        Serial.print(tempFL);
        Serial.print(", FR: ");
        Serial.print(tempFR);
        Serial.print(", BL: ");
        Serial.print(tempBL);
        Serial.print(", BR: ");
        Serial.println(tempBR);
    }
    
    void printRotations() {
        noInterrupts();
        float rotationsFL = countFL / (float)CPR;
        float rotationsFR = countFR / (float)CPR;
        float rotationsBL = countBL / (float)CPR;
        float rotationsBR = countBR / (float)CPR;
        interrupts();
        
        Serial.print("Rotations - FL: ");
        Serial.print(rotationsFL, 2);
        Serial.print(", FR: ");
        Serial.print(rotationsFR, 2);
        Serial.print(", BL: ");
        Serial.print(rotationsBL, 2);
        Serial.print(", BR: ");
        Serial.println(rotationsBR, 2);
    }
    
    long getCountFL() {
        noInterrupts();
        long temp = countFL;
        interrupts();
        return temp;
    }
    
    long getCountFR() {
        noInterrupts();
        long temp = countFR;
        interrupts();
        return temp;
    }
    
    long getCountBL() {
        noInterrupts();
        long temp = countBL;
        interrupts();
        return temp;
    }
    
    long getCountBR() {
        noInterrupts();
        long temp = countBR;
        interrupts();
        return temp;
    }
    
    float getRotationsFL() {
        return getCountFL() / (float)CPR;
    }
    
    float getRotationsFR() {
        return getCountFR() / (float)CPR;
    }
    
    float getRotationsBL() {
        return getCountBL() / (float)CPR;
    }
    
    float getRotationsBR() {
        return getCountBR() / (float)CPR;
    }
    

    // Interrupt service routines
    void ISR_FL() {
        bool stateA = digitalRead(ENC_FLA);
        bool stateB = digitalRead(ENC_FLB);
        
        if (stateA == stateB) {
            countFL--;
        } else {
            countFL++;
        }
    }
    
    void ISR_FR() {
        bool stateA = digitalRead(ENC_FRA);
        bool stateB = digitalRead(ENC_FRB);
        
        if (stateA == stateB) {
            countFR++;
        } else {
            countFR--;
        }
    }
    
    void ISR_BL() {
        bool stateA = digitalRead(ENC_BLA);
        bool stateB = digitalRead(ENC_BLB);
        
        if (stateA == stateB) {
            countBL--;
        } else {
            countBL++;
        }
    }
    
    void ISR_BR() {
        bool stateA = digitalRead(ENC_BRA);
        bool stateB = digitalRead(ENC_BRB);
        
        if (stateA == stateB) {
            countBR++;
        } else {
            countBR--;
        }
    }
}

// ISR Wrappers
void encoderISR_FL() {
    Encoders::ISR_FL();
}

void encoderISR_FR() {
    Encoders::ISR_FR();
}

void encoderISR_BL() {
    Encoders::ISR_BL();
}

void encoderISR_BR() {
    Encoders::ISR_BR();
}
