#ifndef TOF_CAMERAS_H
#define TOF_CAMERAS_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

namespace TOF {
    // Number of sensors
    const int SENSOR_COUNT = 3;

    class MedianFilter {
    private:
        static const int WINDOW_SIZE = 5;
        float readings[WINDOW_SIZE];
        int readIndex;
        int numReadings;

    public:
        MedianFilter();
        float updateEstimate(float measurement);
        void reset();
    };

    class TOFSensors {
    private:
        VL53L0X sensors[SENSOR_COUNT];
        MedianFilter medianFilters[SENSOR_COUNT];
        float sensorOffsets[SENSOR_COUNT] = {0.0};
        int xshutPins[SENSOR_COUNT] = {0};
        uint8_t sensorAddresses[SENSOR_COUNT] = {0};
        uint8_t sdaPin = 0;
        uint8_t sclPin = 0;
        bool configurationSet = false;
        bool initializationAttempted = false;
        bool lastInitSuccessful = false;

    public:
        bool initialize(const int* xshutPinsArray,
                        const uint8_t* sensorAddressArray,
                        uint8_t sda,
                        uint8_t scl);

        bool begin();

        // Core functionality
        uint16_t readSensor(int sensorIndex);
        bool sensorTimeout(int sensorIndex);
        float getFilteredDistanceCM(int sensorIndex);
        void resetFilter(int sensorIndex);
        
        // Calibration
        void setSensorOffset(int sensorIndex, float offsetMM);
        float getSensorOffset(int sensorIndex);
        
        // Getters for direct sensor access if needed
        VL53L0X& getSensor(int index) { return sensors[index]; }
        bool isInitialized() const { return initializationAttempted; }
        bool initializationSucceeded() const { return lastInitSuccessful; }
        bool isConfigured() const { return configurationSet; }
    };
}

#endif // TOF_CAMERAS_H
