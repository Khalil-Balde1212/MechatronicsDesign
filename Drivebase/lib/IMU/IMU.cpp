#include "IMU.h"

static inline float rad2deg_conv(float r) { 
    return r * 57.29577951308232f; 
}

float IMUController::deg2rad(float deg) {
    return deg * 0.017453292519943295f;
}

float IMUController::rad2deg(float rad) {
    return rad * 57.29577951308232f;
}

float IMUController::wrapDeg(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

float IMUController::wrapHeading(float h) {
    while (h >= 360.0f) h -= 360.0f;
    while (h < 0.0f) h += 360.0f;
    return h;
}

bool IMUController::begin() {
    if (!IMU.begin()) {
        initialized = false;
        return false;
    }
    
    initialized = true;
    lastUpdateTime = millis();
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    heading = 0.0f;
    
    return true;
}

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

void IMUController::setAccelScales(float x, float y, float z) {
    accel_scale_x = x;
    accel_scale_y = y;
    accel_scale_z = z;
}

void IMUController::update() {
    if (!initialized) return;

    unsigned long currentTime = millis();
    dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;
    
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.01f;
    }
    
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        ax = (ax - accel_offset_x) * accel_scale_x;
        ay = (ay - accel_offset_y) * accel_scale_y;
        az = (az - accel_offset_z) * accel_scale_z;
    }
    
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
        gx -= gyro_offset_x;
        gy -= gyro_offset_y;
        gz -= gyro_offset_z;
        
        omega_x = gx;
        omega_y = gy;
        omega_z = gz;
        
        alpha_x = (omega_x - prev_omega_x) / dt;
        alpha_y = (omega_y - prev_omega_y) / dt;
        alpha_z = (omega_z - prev_omega_z) / dt;
        
        prev_omega_x = omega_x;
        prev_omega_y = omega_y;
        prev_omega_z = omega_z;
    }
    
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
    }
    
    // Calculate orientation using complementary filter
    float roll_acc = rad2deg(atan2(-ay, az));
    float pitch_acc = rad2deg(atan2(ax, sqrt(ay*ay + az*az)));
    
    // Calculate absolute heading from magnetometer (drift-free!)
    float heading_mag = rad2deg(atan2(my, mx));
    heading_mag = wrapHeading(heading_mag);
    
    // Integrate gyro for high-frequency changes
    float roll_gyro = roll - gx * dt;
    float pitch_gyro = pitch + gy * dt;
    float yaw_gyro = yaw - gz * dt;
    
    // Complementary filter: fuse accel + gyro for roll/pitch
    roll = alpha_filter * roll_gyro + (1.0f - alpha_filter) * roll_acc;
    pitch = alpha_filter * pitch_gyro + (1.0f - alpha_filter) * pitch_acc;
    
    // Complementary filter: fuse magnetometer + gyro for yaw (absolute orientation!)
    // Handle 360° wrap-around for proper fusion
    float yaw_diff = heading_mag - yaw;
    if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
    if (yaw_diff < -180.0f) yaw_diff += 360.0f;
    
    yaw = alpha_filter * yaw_gyro + (1.0f - alpha_filter) * (yaw + yaw_diff);
    
    // Wrap angles
    roll = wrapDeg(roll);
    pitch = wrapDeg(pitch);
    yaw = wrapDeg(yaw);
    
    // Store absolute heading (same as fused yaw)
    heading = heading_mag;
}

void IMUController::resetAngles() {
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    heading = 0.0f;
}