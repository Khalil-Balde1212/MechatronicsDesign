#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Encoders.h"
#include "IMU.h"

#include "CommandInterpretter.h"
#include "RobotMap.h"
#include <Motors.h>

IMUController imuController;

void setup() {
    Serial.begin(9600);

    unsigned long _t0 = millis();
    while (!Serial && (millis() - _t0 < 2000)) { }

    Wire.begin();

    Serial.println("Setup complete - Motor and Encoders ready!");

    CommandInterpreter::begin();
    CommandInterpreter::registerCommand({"ping", [](const std::string* args)
    {
        Serial.println("pong");
    },
    "Usage: ping ## \n Sends 'pong' response. Optionally specify number of times to respond."
    });


    if (!imuController.begin()) {
        Serial.println("[ERROR] IMU initialization failed!");
        while (1);
    }

    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int samples = 0;
    
    // Collect 500 samples silently
    for (int i = 0; i < 500; i++) {
        if (IMU.gyroscopeAvailable()) {
            float gx, gy, gz;
            IMU.readGyroscope(gx, gy, gz);
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            samples++;
        }
        delay(10);
    }
    
    // Apply calibration
    if (samples > 0) {
        imuController.setGyroOffsets(gx_sum / samples, gy_sum / samples, gz_sum / samples);
    }
    
    imuController.resetAngles();
}

void loop()
{
    CommandInterpreter::periodic();

    imuController.update();
    imuController.printAngles();
}

