#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

class IMUController {
private:
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;

    float gyro_offset_x  = 0.0f;
    float gyro_offset_y  = 0.0f;
    float gyro_offset_z  = 0.0f;

    float accel_offset_x = 0.0f;
    float accel_offset_y = 0.0f;
    float accel_offset_z = 0.0f;

    float accel_scale_x  = 1.0f;
    float accel_scale_y  = 1.0f;
    float accel_scale_z  = 1.0f;

    float roll    = 0.0f;
    float pitch   = 0.0f;
    float yaw     = 0.0f;
    float heading = 0.0f;

    float omega_x = 0.0f;
    float omega_y = 0.0f;
    float omega_z = 0.0f;

    float alpha_x = 0.0f;
    float alpha_y = 0.0f;
    float alpha_z = 0.0f;
    
    float prev_omega_x = 0.0f;
    float prev_omega_y = 0.0f;
    float prev_omega_z = 0.0f;

    unsigned long lastUpdateTime = 0;
    float dt = 0.0f;

    const float alpha_filter = 0.98f;
    bool initialized = false;

    static float wrapDeg(float a);
    static float wrapHeading(float h);
    static float deg2rad(float deg);
    static float rad2deg(float rad);

public:
    bool begin();
    void update();
    
    void setGyroOffsets(float x, float y, float z);
    void setAccelOffsets(float x, float y, float z);
    void setAccelScales(float x, float y, float z);
    
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    float getHeading() const { return heading; }
    
    float getRollRad() const { return deg2rad(roll); }
    float getPitchRad() const { return deg2rad(pitch); }
    float getYawRad() const { return deg2rad(yaw); }
    float getHeadingRad() const { return deg2rad(heading); }
    
    float getOmegaX() const { return omega_x; }
    float getOmegaY() const { return omega_y; }
    float getOmegaZ() const { return omega_z; }
    
    float getOmegaXRad() const { return deg2rad(omega_x); }
    float getOmegaYRad() const { return deg2rad(omega_y); }
    float getOmegaZRad() const { return deg2rad(omega_z); }
    
    float getAlphaX() const { return alpha_x; }
    float getAlphaY() const { return alpha_y; }
    float getAlphaZ() const { return alpha_z; }
    
    float getAlphaXRad() const { return deg2rad(alpha_x); }
    float getAlphaYRad() const { return deg2rad(alpha_y); }
    float getAlphaZRad() const { return deg2rad(alpha_z); }
    
    float getAccelX() const { return ax; }
    float getAccelY() const { return ay; }
    float getAccelZ() const { return az; }
    
    float getGyroX() const { return gx; }
    float getGyroY() const { return gy; }
    float getGyroZ() const { return gz; }
    
    float getMagX() const { return mx; }
    float getMagY() const { return my; }
    float getMagZ() const { return mz; }
    
    void resetAngles();
    bool isInitialized() const { return initialized; }
};

#endif // IMU_H