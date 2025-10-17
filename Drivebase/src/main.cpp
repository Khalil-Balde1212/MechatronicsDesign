#include <Arduino.h>
#include "RobotMap.h"
#include "Encoders.h"
#include "Motors.h"
#include "Drivebase.h"


static unsigned long lastPrintMs = 0;
void setup()
{
    Serial.begin(9600);
    DriveBase::begin();
}

void loop()
{
    DriveBase::update();

    unsigned long now = millis();
    if (now - lastPrintMs >= 500) {
        lastPrintMs = now;
        DriveBase::encoderFR.printStatus();
    }
}