#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

class IMUController {
private:
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    bool initialized;

public:
    IMUController();
    bool begin();
    void update();
    
    // Gyroscope functions
    bool readGyroscope();
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }
    
    // Accelerometer functions
    bool readAccelerometer();
    float getAccelX() const { return accelX; }
    float getAccelY() const { return accelY; }
    float getAccelZ() const { return accelZ; }
    
    // Utility functions
    bool isInitialized() const { return initialized; }
    void printGyroData();
    void printAccelData();
    void printAllData();
};

#endif