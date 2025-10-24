#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

class IMUController {
private:
    // ---- Calibrated sensor readings ----
    float ax = 0.0f, ay = 0.0f, az = 0.0f;   // Accelerometer (g units)
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;   // Gyroscope (deg/s)

    // ---- Calibration parameters ----
    float gyro_offset_x  = 0.0f;
    float gyro_offset_y  = 0.0f;
    float gyro_offset_z  = 0.0f;

    float accel_offset_x = 0.0f;
    float accel_offset_y = 0.0f;
    float accel_offset_z = 0.0f;

    float accel_scale_x  = 1.0f;
    float accel_scale_y  = 1.0f;
    float accel_scale_z  = 1.0f;

    // ---- Fused orientation angles ----
    float roll  = 0.0f;  // degrees
    float pitch = 0.0f;  // degrees
    float yaw   = 0.0f;  // degrees

    // ---- Angular velocity (from gyro, deg/s) ----
    float omega_x = 0.0f;  // Roll rate
    float omega_y = 0.0f;  // Pitch rate
    float omega_z = 0.0f;  // Yaw rate

    // ---- Angular acceleration (deg/s²) ----
    float alpha_x = 0.0f;
    float alpha_y = 0.0f;
    float alpha_z = 0.0f;
    
    // Previous angular velocities for acceleration calculation
    float prev_omega_x = 0.0f;
    float prev_omega_y = 0.0f;
    float prev_omega_z = 0.0f;

    // ---- Timing ----
    unsigned long lastUpdateTime = 0;
    float dt = 0.0f;

    // ---- Complementary filter weight ----
    const float alpha_filter = 0.98f;
    
    // ---- Yaw correction factor (calibrated: 20° measured = 90° actual) ----
    const float yaw_scale = 4.5f;

    bool initialized = false;

    // Helper functions
    static float wrapDeg(float a);
    static float deg2rad(float deg);

public:
    // Init
    bool begin();
    
    // Update (call every loop)
    void update();
    
    // Calibration setters
    void setGyroOffsets(float x, float y, float z);
    void setAccelOffsets(float x, float y, float z);
    void setAccelScales(float x, float y, float z);
    
    // Angle getters (degrees and radians)
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    
    float getRollRad() const { return deg2rad(roll); }
    float getPitchRad() const { return deg2rad(pitch); }
    float getYawRad() const { return deg2rad(yaw); }
    
    // Angular velocity getters (deg/s and rad/s)
    float getOmegaX() const { return omega_x; }
    float getOmegaY() const { return omega_y; }
    float getOmegaZ() const { return omega_z; }
    
    float getOmegaXRad() const { return deg2rad(omega_x); }
    float getOmegaYRad() const { return deg2rad(omega_y); }
    float getOmegaZRad() const { return deg2rad(omega_z); }
    
    // Angular acceleration getters (deg/s² and rad/s²)
    float getAlphaX() const { return alpha_x; }
    float getAlphaY() const { return alpha_y; }
    float getAlphaZ() const { return alpha_z; }
    
    float getAlphaXRad() const { return deg2rad(alpha_x); }
    float getAlphaYRad() const { return deg2rad(alpha_y); }
    float getAlphaZRad() const { return deg2rad(alpha_z); }
    
    // Raw sensor getters (calibrated)
    float getAccelX() const { return ax; }
    float getAccelY() const { return ay; }
    float getAccelZ() const { return az; }
    
    float getGyroX() const { return gx; }
    float getGyroY() const { return gy; }
    float getGyroZ() const { return gz; }
    
    // Print functions
    void printAngles();
    void printAnglesRad();
    void printAngularVelocity();
    void printAngularVelocityRad();
    void printAngularAcceleration();
    void printAngularAccelerationRad();
    void printGyroData();
    void printAccelData();
    void printStatus();
    void printCompleteSummary();  // NEW: All data formatted
    
    // Utility
    void resetAngles();
    bool isInitialized() const { return initialized; }
};

#endif // IMU_H