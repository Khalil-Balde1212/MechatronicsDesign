#include "IMU.h"

static inline float rad2deg_conv(float r) { 
    return r * 57.29577951308232f; 
}

float IMUController::deg2rad(float deg) {
    return deg * 0.017453292519943295f;  // deg * PI/180
}

float IMUController::wrapDeg(float a) {
    while (a >= 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

bool IMUController::begin() {
    if (!IMU.begin()) {
        Serial.println("[IMU ERROR] Failed to initialize LSM6DS3!");
        initialized = false;
        return false;
    }
    
    Serial.println("[IMU] LSM6DS3 initialized");
    Serial.print("[IMU] Accel rate: ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.print("[IMU] Gyro rate: ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    
    lastUpdateTime = millis();
    initialized = true;

    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int samples = 0;
    
    for (int i = 0; i < 1000; i++) {
        if (IMU.gyroscopeAvailable()) {
            float gx, gy, gz;
            IMU.readGyroscope(gx, gy, gz);
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            samples++;
        }
    }
    
    // Apply calibration
    if (samples > 0) {
        setGyroOffsets(gx_sum / samples, gy_sum / samples, gz_sum / samples);
    }
    
    resetAngles();

    return true;
}

void IMUController::setGyroOffsets(float x, float y, float z) {
    gyro_offset_x = x;
    gyro_offset_y = y;
    gyro_offset_z = z;
    Serial.println("[IMU] Gyro offsets set:");
    Serial.print("  X: "); Serial.print(x, 3); Serial.println(" deg/s");
    Serial.print("  Y: "); Serial.print(y, 3); Serial.println(" deg/s");
    Serial.print("  Z: "); Serial.print(z, 3); Serial.println(" deg/s");
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

    // Calculate time step
    unsigned long now = millis();
    dt = (now - lastUpdateTime) * 0.001f;
    if (dt < 0.0f || dt > 0.5f) dt = 0.0f;
    lastUpdateTime = now;

    bool gotAccel = IMU.accelerationAvailable();
    bool gotGyro  = IMU.gyroscopeAvailable();

    // Read and calibrate accelerometer
    if (gotAccel) {
        float axr, ayr, azr;
        IMU.readAcceleration(axr, ayr, azr);
        ax = (axr - accel_offset_x) * accel_scale_x;
        ay = (ayr - accel_offset_y) * accel_scale_y;
        az = (azr - accel_offset_z) * accel_scale_z;
    }

    // Read and calibrate gyroscope
    if (gotGyro) {
        float gxr, gyr, gzr;
        IMU.readGyroscope(gxr, gyr, gzr);
        gx = gxr - gyro_offset_x;
        gy = gyr - gyro_offset_y;
        gz = gzr - gyro_offset_z;
        
        // Update angular velocities (gyro readings ARE angular velocities)
        omega_x = gx;  // Roll rate (deg/s)
        omega_y = gy;  // Pitch rate (deg/s)
        omega_z = gz;  // Yaw rate (deg/s) - uncorrected, raw value
        
        // Calculate angular accelerations (change in angular velocity / time)
        if (dt > 0.0f) {
            alpha_x = (omega_x - prev_omega_x) / dt;  // deg/s²
            alpha_y = (omega_y - prev_omega_y) / dt;
            alpha_z = (omega_z - prev_omega_z) / dt;
            
            // Store for next iteration
            prev_omega_x = omega_x;
            prev_omega_y = omega_y;
            prev_omega_z = omega_z;
        }
    }

    // Complementary filter
    if (dt > 0.0f) {
        if (gotAccel && gotGyro) {
            // Calculate angles from accelerometer
            float rollAcc  = rad2deg_conv(atan2f(-ay, az));
            float pitchAcc = rad2deg_conv(atan2f(ax, sqrtf(ay*ay + az*az)));

            // Apply yaw scale correction
            float gz_scaled = gz * yaw_scale;
            
            // Integrate gyroscope
            float rollGyro  = roll  - gx * dt;
            float pitchGyro = pitch + gy * dt;
            float yawGyro   = yaw   - gz_scaled * dt;

            // Fuse with complementary filter
            roll  = alpha_filter * rollGyro  + (1.0f - alpha_filter) * rollAcc;
            pitch = alpha_filter * pitchGyro + (1.0f - alpha_filter) * pitchAcc;
            yaw   = yawGyro;

            // Wrap angles
            roll  = wrapDeg(roll);
            pitch = wrapDeg(pitch);
            yaw   = wrapDeg(yaw);
        } else if (gotGyro) {
            // Only gyro available
            float gz_scaled = gz * yaw_scale;
            
            roll  = wrapDeg(roll  - gx * dt);
            pitch = wrapDeg(pitch + gy * dt);
            yaw   = wrapDeg(yaw   - gz_scaled * dt);
        }
    }
}

void IMUController::printAngles() {
    Serial.print("Angles[deg] Roll=");
    Serial.print(roll, 2);
    Serial.print(" Pitch=");
    Serial.print(pitch, 2);
    Serial.print(" Yaw=");
    Serial.println(yaw, 2);
}

void IMUController::printAnglesRad() {
    Serial.print("Angles[rad] Roll=");
    Serial.print(getRollRad(), 4);
    Serial.print(" Pitch=");
    Serial.print(getPitchRad(), 4);
    Serial.print(" Yaw=");
    Serial.println(getYawRad(), 4);
}

void IMUController::printAngularVelocity() {
    Serial.print("AngVel[deg/s] ωx=");
    Serial.print(omega_x, 2);
    Serial.print(" ωy=");
    Serial.print(omega_y, 2);
    Serial.print(" ωz=");
    Serial.println(omega_z, 2);
}

void IMUController::printAngularVelocityRad() {
    Serial.print("AngVel[rad/s] ωx=");
    Serial.print(getOmegaXRad(), 4);
    Serial.print(" ωy=");
    Serial.print(getOmegaYRad(), 4);
    Serial.print(" ωz=");
    Serial.println(getOmegaZRad(), 4);
}

void IMUController::printAngularAcceleration() {
    Serial.print("AngAcc[deg/s²] αx=");
    Serial.print(alpha_x, 2);
    Serial.print(" αy=");
    Serial.print(alpha_y, 2);
    Serial.print(" αz=");
    Serial.println(alpha_z, 2);
}

void IMUController::printAngularAccelerationRad() {
    Serial.print("AngAcc[rad/s²] αx=");
    Serial.print(getAlphaXRad(), 4);
    Serial.print(" αy=");
    Serial.print(getAlphaYRad(), 4);
    Serial.print(" αz=");
    Serial.println(getAlphaZRad(), 4);
}

void IMUController::printGyroData() {
    Serial.print("Gyro[dps] gx=");
    Serial.print(gx, 2);
    Serial.print(" gy=");
    Serial.print(gy, 2);
    Serial.print(" gz=");
    Serial.println(gz, 2);
}

void IMUController::printAccelData() {
    Serial.print("Accel[g] ax=");
    Serial.print(ax, 3);
    Serial.print(" ay=");
    Serial.print(ay, 3);
    Serial.print(" az=");
    Serial.println(az, 3);
}

void IMUController::printStatus() {
    Serial.print("Roll=");
    Serial.print(roll, 1);
    Serial.print("° Pitch=");
    Serial.print(pitch, 1);
    Serial.print("° Yaw=");
    Serial.print(yaw, 1);
    Serial.print("° | Gyro: X=");
    Serial.print(gx, 2);
    Serial.print(" Y=");
    Serial.print(gy, 2);
    Serial.print(" Z=");
    Serial.print(gz, 2);
    Serial.print(" | Accel: X=");
    Serial.print(ax, 3);
    Serial.print(" Y=");
    Serial.print(ay, 3);
    Serial.print(" Z=");
    Serial.println(az, 3);
}

void IMUController::printCompleteSummary() {
    Serial.println("\n╔════════════════════ IMU COMPLETE DATA ════════════════════╗");
    
    // Orientation Angles
    Serial.println("║ ORIENTATION ANGLES                                        ║");
    Serial.print("║   Roll:  ");
    Serial.print(roll, 2);
    Serial.print("° (");
    Serial.print(getRollRad(), 4);
    Serial.println(" rad)");
    
    Serial.print("║   Pitch: ");
    Serial.print(pitch, 2);
    Serial.print("° (");
    Serial.print(getPitchRad(), 4);
    Serial.println(" rad)");
    
    Serial.print("║   Yaw:   ");
    Serial.print(yaw, 2);
    Serial.print("° (");
    Serial.print(getYawRad(), 4);
    Serial.println(" rad)");
    
    // Angular Velocities
    Serial.println("║                                                           ║");
    Serial.println("║ ANGULAR VELOCITIES                                        ║");
    Serial.print("║   ωx: ");
    Serial.print(omega_x, 2);
    Serial.print(" deg/s (");
    Serial.print(getOmegaXRad(), 4);
    Serial.println(" rad/s)");
    
    Serial.print("║   ωy: ");
    Serial.print(omega_y, 2);
    Serial.print(" deg/s (");
    Serial.print(getOmegaYRad(), 4);
    Serial.println(" rad/s)");
    
    Serial.print("║   ωz: ");
    Serial.print(omega_z, 2);
    Serial.print(" deg/s (");
    Serial.print(getOmegaZRad(), 4);
    Serial.println(" rad/s)");
    
    // Angular Accelerations
    Serial.println("║                                                           ║");
    Serial.println("║ ANGULAR ACCELERATIONS                                     ║");
    Serial.print("║   αx: ");
    Serial.print(alpha_x, 2);
    Serial.print(" deg/s² (");
    Serial.print(getAlphaXRad(), 4);
    Serial.println(" rad/s²)");
    
    Serial.print("║   αy: ");
    Serial.print(alpha_y, 2);
    Serial.print(" deg/s² (");
    Serial.print(getAlphaYRad(), 4);
    Serial.println(" rad/s²)");
    
    Serial.print("║   αz: ");
    Serial.print(alpha_z, 2);
    Serial.print(" deg/s² (");
    Serial.print(getAlphaZRad(), 4);
    Serial.println(" rad/s²)");
    
    // Raw Sensors
    Serial.println("║                                                           ║");
    Serial.println("║ RAW SENSORS                                               ║");
    Serial.print("║   Accel: X=");
    Serial.print(ax, 3);
    Serial.print("g Y=");
    Serial.print(ay, 3);
    Serial.print("g Z=");
    Serial.print(az, 3);
    Serial.println("g");
    
    Serial.print("║   Gyro:  X=");
    Serial.print(gx, 2);
    Serial.print("°/s Y=");
    Serial.print(gy, 2);
    Serial.print("°/s Z=");
    Serial.print(gz, 2);
    Serial.println("°/s");
    
    Serial.println("╚═══════════════════════════════════════════════════════════╝\n");
}

void IMUController::resetAngles() {
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    Serial.println("[IMU] Angles reset to zero");
}