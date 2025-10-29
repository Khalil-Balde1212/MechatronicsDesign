#include "IMU.h"
#include <Arduino_BMI270_BMM150.h>

bool IMUController::begin() {
    if (!IMU.begin()) {
        initialized = false;
        return false;
    }

    initialized = true;
    lastUpdateTime = millis();

    // Initialize quaternion (no rotation)
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    return true;
}

void IMUController::update() {
    if (!initialized) return;

    unsigned long currentTime = millis();
    dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;

    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.01f; // Default to 10ms if timing is off
    }

    // Read sensor data
    bool accelAvailable = IMU.accelerationAvailable();
    bool gyroAvailable = IMU.gyroscopeAvailable();
    bool magAvailable = IMU.magneticFieldAvailable();

    if (accelAvailable) {
        IMU.readAcceleration(ax, ay, az);
        // Apply calibration
        ax = (ax - accel_offset_x) * accel_scale_x;
        ay = (ay - accel_offset_y) * accel_scale_y;
        az = (az - accel_offset_z) * accel_scale_z;
    }

    if (gyroAvailable) {
        IMU.readGyroscope(gx, gy, gz);
        // Convert to radians/second and apply calibration
        gx = (gx - gyro_offset_x) * DEG_TO_RAD;
        gy = (gy - gyro_offset_y) * DEG_TO_RAD;
        gz = (gz - gyro_offset_z) * DEG_TO_RAD;
    }

    if (magAvailable) {
        IMU.readMagneticField(mx, my, mz);
        // Apply calibration
        mx = (mx - mag_offset_x) * mag_scale_x;
        my = (my - mag_offset_y) * mag_scale_y;
        mz = (mz - mag_offset_z) * mag_scale_z;
    }

    // Run Madgwick filter
    if (accelAvailable && gyroAvailable) {
        if (magAvailable) {
            // Serial.println("Madgwick filter: Magnetometer USED");
            madgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz);
        } else {
            // Serial.println("Madgwick filter: Magnetometer NOT used");
            madgwickUpdateIMU(gx, gy, gz, ax, ay, az);
        }
    }
    // Artificially increment heading offset by 0.1 degrees every update
    headingOffset += 0.005f;
}

// Fast inverse square root approximation
float IMUController::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// Madgwick 9DOF filter (with magnetometer)
void IMUController::madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        // Normalise step magnitude
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// Madgwick 6DOF filter (IMU only, no magnetometer)
void IMUController::madgwickUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        // Normalise step magnitude
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// Convert quaternion to Euler angles
void IMUController::quaternionToEuler(float& roll, float& pitch, float& yaw) const {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (q0 * q2 - q3 * q1);
    if (abs(sinp) >= 1)
        pitch = copysign(PI / 2, sinp); // Use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    yaw = atan2(siny_cosp, cosy_cosp);
}

// Get Euler angles in degrees
float IMUController::getRoll() const {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    return roll * RAD_TO_DEG;
}

float IMUController::getPitch() const {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    return pitch * RAD_TO_DEG;
}

float IMUController::getYaw() const {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    return yaw * RAD_TO_DEG;
}

float IMUController::getHeading() const {
    // Heading is the same as yaw for navigation purposes
    return getYaw() + headingOffset;
}

// Get Euler angles in radians
float IMUController::getRollRad() const {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    return roll;
}

float IMUController::getPitchRad() const {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    return pitch;
}

float IMUController::getYawRad() const {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    return yaw;
}

float IMUController::getHeadingRad() const {
    return getYawRad();
}

// Calibration setters
void IMUController::setGyroOffsets(float x, float y, float z) {
    gyro_offset_x = x;
    gyro_offset_y = y;
    gyro_offset_z = z;
}

void IMUController::setAccelOffsets(float x, float y, float z) {
    accel_offset_x = x;
    accel_offset_y = y;
    accel_offset_z = z;
}

void IMUController::setMagOffsets(float x, float y, float z) {
    mag_offset_x = x;
    mag_offset_y = y;
    mag_offset_z = z;
}

void IMUController::setAccelScales(float x, float y, float z) {
    accel_scale_x = x;
    accel_scale_y = y;
    accel_scale_z = z;
}

void IMUController::setMagScales(float x, float y, float z) {
    mag_scale_x = x;
    mag_scale_y = y;
    mag_scale_z = z;
}

void IMUController::setMadgwickBeta(float b) {
    beta = b;
}

void IMUController::reset() {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    lastUpdateTime = millis();
}

void IMUController::calibrate() {
    Serial.println("Starting IMU calibration. Please keep the robot stationary.");
    calibrateGyro();
    // Optionally add accelerometer and magnetometer calibration here
    // calibrateAccel();
    // calibrateMag();
    Serial.println("IMU calibration complete.");
}

void IMUController::calibrateGyro() {
    const int numSamples = 500;
    float sumX = 0, sumY = 0, sumZ = 0;
    int samples = 0;
    Serial.println("Calibrating gyro... Keep robot stationary.");
    for (int i = 0; i < numSamples; ++i) {
        while (!IMU.gyroscopeAvailable()) {}
        float x, y, z;
        IMU.readGyroscope(x, y, z);
        sumX += x;
        sumY += y;
        sumZ += z;
        samples++;
        delay(2);
    }
    gyro_offset_x = sumX / samples;
    gyro_offset_y = sumY / samples;
    gyro_offset_z = sumZ / samples;
    Serial.print("Gyro offsets set: x=");
    Serial.print(gyro_offset_x);
    Serial.print(", y=");
    Serial.print(gyro_offset_y);
    Serial.print(", z=");
    Serial.println(gyro_offset_z);
}
