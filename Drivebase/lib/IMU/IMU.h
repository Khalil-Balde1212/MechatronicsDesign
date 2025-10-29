#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

class IMUController {
private:
    // Sensor data
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;

    // Madgwick filter variables
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Quaternion
    float beta = 0.1f; // Madgwick filter gain
    unsigned long lastUpdateTime = 0;
    float dt = 0.0f;

    // Calibration offsets
    float gyro_offset_x = 0.0f;
    float gyro_offset_y = 0.0f;
    float gyro_offset_z = 0.0f;
    float accel_offset_x = 0.0f;
    float accel_offset_y = 0.0f;
    float accel_offset_z = 0.0f;
    float mag_offset_x = 0.0f;
    float mag_offset_y = 0.0f;
    float mag_offset_z = 0.0f;

    // Scaling factors
    float accel_scale_x = 1.0f;
    float accel_scale_y = 1.0f;
    float accel_scale_z = 1.0f;
    float mag_scale_x = 1.0f;
    float mag_scale_y = 1.0f;
    float mag_scale_z = 1.0f;

    bool initialized = false;

    // Madgwick filter functions
    void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void madgwickUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    // Helper functions
    static float invSqrt(float x);
    void quaternionToEuler(float& roll, float& pitch, float& yaw) const;

public:
    bool begin();
    void update();

    // Calibration functions
    void setGyroOffsets(float x, float y, float z);
    void setAccelOffsets(float x, float y, float z);
    void setMagOffsets(float x, float y, float z);
    void setAccelScales(float x, float y, float z);
    void setMagScales(float x, float y, float z);
    void setMadgwickBeta(float beta);

    // Get orientation in degrees
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
    float getHeading() const;

    // Get orientation in radians
    float getRollRad() const;
    float getPitchRad() const;
    float getYawRad() const;
    float getHeadingRad() const;

    // Get raw sensor data
    float getAccelX() const { return ax; }
    float getAccelY() const { return ay; }
    float getAccelZ() const { return az; }
    float getGyroX() const { return gx; }
    float getGyroY() const { return gy; }
    float getGyroZ() const { return gz; }
    float getMagX() const { return mx; }
    float getMagY() const { return my; }
    float getMagZ() const { return mz; }

    // Get quaternion components
    float getQ0() const { return q0; }
    float getQ1() const { return q1; }
    float getQ2() const { return q2; }
    float getQ3() const { return q3; }

    void reset();
    bool isInitialized() const { return initialized; }
};

#endif // IMU_H
