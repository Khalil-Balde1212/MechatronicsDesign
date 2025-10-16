#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

class IMUController {
private:
    // Latest bias-corrected & smoothed readings
    float gyroX, gyroY, gyroZ;     // deg/s
    float accelX, accelY, accelZ;  // g

    // Raw (internal)
    float gyroX_raw, gyroY_raw, gyroZ_raw;
    float accelX_raw, accelY_raw, accelZ_raw;

    // Gyro bias (deg/s)
    float gyroBiasX, gyroBiasY, gyroBiasZ;

    // EMA smoothing state
    float gxf, gyf, gzf;
    float axf, ayf, azf;
    bool  filtInit;
    float alphaGyro;    // 0..1 (smaller = smoother)
    float alphaAccel;   // 0..1

    // Axis inversion flags
    bool invertGyroZ;
    bool invertAccelZ;

    bool initialized;

    // Internal helpers
    bool readGyroscopeRaw();
    bool readAccelerometerRaw();
    void applyGyroBias();
    void smoothGyro();
    void smoothAccel();

public:
    IMUController();
    bool begin(uint16_t calib_ms = 2000); // 2 s calibration at startup (blocking)
    void update();

    // Gyroscope (bias-corrected & smoothed)
    bool  readGyroscope();
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }

    // Accelerometer (smoothed)
    bool  readAccelerometer();
    float getAccelX() const { return accelX; }
    float getAccelY() const { return accelY; }
    float getAccelZ() const { return accelZ; }

    // Smoothing control
    void setSmoothing(float gyroAlpha, float accelAlpha);

    // Calibration utilities
    bool calibrateGyro(uint16_t calib_ms = 2000);
    void tareGyro();
    void setGyroBias(float bx, float by, float bz);

    // Optional Z flip
    void setFlipZ(bool flipGyroZ, bool flipAccelZ);

    // Config helpers (Accel ±4 g, Gyro ±2000 dps, ODR 104 Hz)
    void forceDefaultImuConfig();
    void printImuConfig();

    bool isInitialized() const { return initialized; }
    void printGyroData();
    void printAccelData();
    void printAllData();
};

#endif

