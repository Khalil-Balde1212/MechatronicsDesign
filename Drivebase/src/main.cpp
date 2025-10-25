#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Encoders.h"
#include "IMU.h"

#include "CommandInterpretter.h"
#include "RobotMap.h"
#include <Motors.h>

#include "Drivebase.h"

void setup() {
    Serial.begin(9600);

    Wire.begin();

    Serial.println("Setup complete - Motor and Encoders ready!");

    CommandInterpreter::begin();
    DriveBase::begin();
}

void loop()
{
    CommandInterpreter::periodic();
    DriveBase::update();
}

