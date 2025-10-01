#include "tof_cameras.h"

namespace TOF {
    bool TOFSensors::initialize(const int* xshutPinsArray,
                                 const uint8_t* sensorAddressArray,
                                 uint8_t sda,
                                 uint8_t scl) {
        if (!xshutPinsArray || !sensorAddressArray) {
            configurationSet = false;
            initializationAttempted = false;
            lastInitSuccessful = false;
            return false;
        }

        for (int i = 0; i < SENSOR_COUNT; i++) {
            xshutPins[i] = xshutPinsArray[i];
            sensorAddresses[i] = sensorAddressArray[i];
            medianFilters[i].reset();
            sensorOffsets[i] = 0.0f;
        }

        sdaPin = sda;
        sclPin = scl;
        configurationSet = true;
        initializationAttempted = false;
        lastInitSuccessful = false;
        return true;
    }

    bool TOFSensors::begin() {
        if (!configurationSet) {
            Serial.println("TOF::TOFSensors - call initialize() before begin().");
            initializationAttempted = false;
            lastInitSuccessful = false;
            return false;
        }

        // Initialise I2C with optional custom pins when supported by the core
#if defined(ARDUINO_ARCH_SAMD)
        if (sdaPin == 0 && sclPin == 0) {
            Wire.begin();
        } else {
            Wire.begin(sdaPin, sclPin);
        }
#else
        Wire.begin();
#endif

        // Prepare all XSHUT lines in reset
        for (int i = 0; i < SENSOR_COUNT; i++) {
            pinMode(xshutPins[i], OUTPUT);
            digitalWrite(xshutPins[i], LOW);
        }

        delay(100);

        bool allSensorsOk = true;

        // Bring sensors up one at a time and configure addresses
        for (int i = 0; i < SENSOR_COUNT; i++) {
            digitalWrite(xshutPins[i], HIGH);
            delay(50);

            VL53L0X& sensor = sensors[i];
            sensor.setTimeout(500);

            if (!sensor.init()) {
                Serial.print("Failed to detect and initialize sensor ");
                Serial.println(i + 1);
                digitalWrite(xshutPins[i], LOW);
                allSensorsOk = false;
                continue;
            }

            sensor.setAddress(sensorAddresses[i]);
            sensor.setMeasurementTimingBudget(50000);

            medianFilters[i].reset();
            sensorOffsets[i] = 0.0f;

            Serial.print("Sensor ");
            Serial.print(i + 1);
            Serial.println(" initialized successfully!");
        }

        initializationAttempted = true;
        lastInitSuccessful = allSensorsOk;

        if (allSensorsOk) {
            Serial.println("All TOF sensors initialized!");
        } else {
            Serial.println("TOF sensor initialization completed with errors.");
        }

        return allSensorsOk;
    }
    // MedianFilter Implementation
    MedianFilter::MedianFilter() {
        readIndex = 0;
        numReadings = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            readings[i] = 0;
        }
    }

    float MedianFilter::updateEstimate(float measurement) {
        readings[readIndex] = measurement;
        readIndex = (readIndex + 1) % WINDOW_SIZE;
        if (numReadings < WINDOW_SIZE) numReadings++;
        
        // Create sorted copy
        float sorted[WINDOW_SIZE];
        for (int i = 0; i < numReadings; i++) {
            sorted[i] = readings[i];
        }
        
        // Simple insertion sort for small array
        for (int i = 1; i < numReadings; i++) {
            float key = sorted[i];
            int j = i - 1;
            while (j >= 0 && sorted[j] > key) {
                sorted[j + 1] = sorted[j];
                j--;
            }
            sorted[j + 1] = key;
        }
        
        return sorted[numReadings / 2];
    }

    void MedianFilter::reset() {
        readIndex = 0;
        numReadings = 0;
    }

    // TOFSensors Implementation
    uint16_t TOFSensors::readSensor(int sensorIndex) {
        if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
            return sensors[sensorIndex].readRangeSingleMillimeters();
        }
        return 0;
    }

    bool TOFSensors::sensorTimeout(int sensorIndex) {
        if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
            return sensors[sensorIndex].timeoutOccurred();
        }
        return true;
    }

    float TOFSensors::getFilteredDistanceCM(int sensorIndex) {
        if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
            uint16_t rawDistance = readSensor(sensorIndex);
            if (sensorTimeout(sensorIndex)) {
                return -1;
            }
            // Apply offset correction and convert to cm
            float correctedMM = rawDistance - sensorOffsets[sensorIndex];
            float rawCM = correctedMM / 10.0;
            return medianFilters[sensorIndex].updateEstimate(rawCM);
        }
        return -1;
    }

    void TOFSensors::resetFilter(int sensorIndex) {
        if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
            medianFilters[sensorIndex].reset();
        }
    }

    void TOFSensors::setSensorOffset(int sensorIndex, float offsetMM) {
        if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
            sensorOffsets[sensorIndex] = offsetMM;
        }
    }

    float TOFSensors::getSensorOffset(int sensorIndex) {
        if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
            return sensorOffsets[sensorIndex];
        }
        return 0.0;
    }
} // namespace TOF

