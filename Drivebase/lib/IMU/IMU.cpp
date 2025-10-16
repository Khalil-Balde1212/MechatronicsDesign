#include <Arduino.h>  
#include <Wire.h>
#include <math.h>      // for fabsf
#include "IMU.h"

// LSM6DS3 I2C address and key registers (Nano 33 IoT uses 0x6A)
static const uint8_t LSM6DS3_ADDR = 0x6A;
static const uint8_t REG_CTRL1_XL = 0x10;  // Accel: ODR_XL[3:0], FS_XL[1:0]
static const uint8_t REG_CTRL2_G  = 0x11;  // Gyro : ODR_G[3:0],  FS_G[1:0], FS_125

static uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)LSM6DS3_ADDR, 1);
    return Wire.available() ? Wire.read() : 0;
}

static void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

IMUController::IMUController() {
    gyroX = gyroY = gyroZ = 0.0f;
    accelX = accelY = accelZ = 0.0f;

    gyroX_raw = gyroY_raw = gyroZ_raw = 0.0f;
    accelX_raw = accelY_raw = accelZ_raw = 0.0f;

    gyroBiasX = gyroBiasY = gyroBiasZ = 0.0f;

    gxf = gyf = gzf = 0.0f;
    axf = ayf = azf = 0.0f;
    filtInit   = false;

    // Stronger smoothing by default (quieter at rest)
    alphaGyro  = 0.12f;
    alphaAccel = 0.10f;

    invertGyroZ  = false;
    invertAccelZ = false;

    initialized = false;
}

void IMUController::setSmoothing(float gyroAlpha, float accelAlpha) {
    alphaGyro  = constrain(gyroAlpha,  0.0f, 1.0f);
    alphaAccel = constrain(accelAlpha, 0.0f, 1.0f);
}

bool IMUController::begin(uint16_t calib_ms) {
    Wire.begin();
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        initialized = false;
        return false;
    }
    initialized = true;
    Serial.println("IMU initialized successfully.");

    // Enforce desired configuration (matches Arduino_LSM6DS3 conversion)
    forceDefaultImuConfig();
    printImuConfig();

    // Startup gyro calibration (keep board still)
    if (!calibrateGyro(calib_ms)) {
        Serial.println("Warning: Gyro calibration had too few samples. Bias left at 0.");
    }
    return true;
}

bool IMUController::calibrateGyro(uint16_t calib_ms) {
    if (!initialized) return false;

    Serial.print("Calibrating gyro (keep board still) for ");
    Serial.print(calib_ms);
    Serial.println(" ms...");

    uint32_t t0 = millis();
    uint32_t n = 0;
    double sx = 0, sy = 0, sz = 0;

    while (millis() - t0 < calib_ms) {
        if (IMU.gyroscopeAvailable()) {
            float x, y, z;
            IMU.readGyroscope(x, y, z); // deg/s
            sx += x; sy += y; sz += z; n++;
        }
        delay(2);
    }

    if (n > 10) {
        gyroBiasX = sx / n;
        gyroBiasY = sy / n;
        gyroBiasZ = sz / n;
        Serial.print("Gyro bias set (deg/s): ");
        Serial.print(gyroBiasX, 4); Serial.print(", ");
        Serial.print(gyroBiasY, 4); Serial.print(", ");
        Serial.println(gyroBiasZ, 4);
        filtInit = false;  // seed EMA after new bias on next sample
        return true;
    }
    return false;
}

void IMUController::tareGyro() {
    if (!initialized) return;
    float x = 0, y = 0, z = 0;
    if (IMU.gyroscopeAvailable()) IMU.readGyroscope(x, y, z);
    gyroBiasX = x; gyroBiasY = y; gyroBiasZ = z;
    Serial.print("Gyro tared (deg/s): ");
    Serial.print(gyroBiasX, 4); Serial.print(", ");
    Serial.print(gyroBiasY, 4); Serial.print(", ");
    Serial.println(gyroBiasZ, 4);
    filtInit = false;
}

void IMUController::setGyroBias(float bx, float by, float bz) {
    gyroBiasX = bx; gyroBiasY = by; gyroBiasZ = bz;
    filtInit = false;
}

void IMUController::setFlipZ(bool flipGyroZ, bool flipAccelZ) {
    invertGyroZ  = flipGyroZ;
    invertAccelZ = flipAccelZ;
    filtInit = false;
}

bool IMUController::readGyroscopeRaw() {
    if (!initialized) return false;
    if (!IMU.gyroscopeAvailable()) return false;
    IMU.readGyroscope(gyroX_raw, gyroY_raw, gyroZ_raw); // deg/s
    return true;
}

bool IMUController::readAccelerometerRaw() {
    if (!initialized) return false;
    if (!IMU.accelerationAvailable()) return false;
    IMU.readAcceleration(accelX_raw, accelY_raw, accelZ_raw); // g
    return true;
}

void IMUController::applyGyroBias() {
    gyroX = gyroX_raw - gyroBiasX;
    gyroY = gyroY_raw - gyroBiasY;
    gyroZ = gyroZ_raw - gyroBiasZ;
}

void IMUController::smoothGyro() {
    // Apply Z inversion before smoothing so filter state matches output
    float sx = gyroX;
    float sy = gyroY;
    float sz = invertGyroZ ? -gyroZ : gyroZ;

    if (!filtInit) { gxf = sx; gyf = sy; gzf = sz; }
    else {
        gxf += alphaGyro * (sx - gxf);
        gyf += alphaGyro * (sy - gyf);
        gzf += alphaGyro * (sz - gzf);
    }
    gyroX = gxf; gyroY = gyf; gyroZ = gzf;

    // Small deadband to clamp micro-jitter at rest (~60 mdps)
    const float db = 0.06f;
    if (fabsf(gyroX) < db) gyroX = 0.0f;
    if (fabsf(gyroY) < db) gyroY = 0.0f;
    if (fabsf(gyroZ) < db) gyroZ = 0.0f;
}

void IMUController::smoothAccel() {
    // Apply Z inversion before smoothing
    float rx = accelX_raw;
    float ry = accelY_raw;
    float rz = invertAccelZ ? -accelZ_raw : accelZ_raw;

    if (!filtInit) { axf = rx; ayf = ry; azf = rz; filtInit = true; }
    else {
        axf += alphaAccel * (rx - axf);
        ayf += alphaAccel * (ry - ayf);
        azf += alphaAccel * (rz - azf);
    }
    accelX = axf; accelY = ayf; accelZ = azf;
}

void IMUController::update() {
    if (!initialized) return;

    if (readGyroscopeRaw()) {
        applyGyroBias();
        smoothGyro();
    }
    if (readAccelerometerRaw()) {
        smoothAccel();
    }
}

bool IMUController::readGyroscope() {
    if (!readGyroscopeRaw()) return false;
    applyGyroBias();
    smoothGyro();
    return true;
}

bool IMUController::readAccelerometer() {
    if (!readAccelerometerRaw()) return false;
    smoothAccel();
    return true;
}

void IMUController::printGyroData() {
    Serial.print("Gyro (deg/s): X=");
    Serial.print(gyroX, 3);
    Serial.print("\tY=");
    Serial.print(gyroY, 3);
    Serial.print("\tZ=");
    Serial.println(gyroZ, 3);
}

void IMUController::printAccelData() {
    Serial.print("Accel (g):   X=");
    Serial.print(accelX, 3);
    Serial.print("\tY=");
    Serial.print(accelY, 3);
    Serial.print("\tZ=");
    Serial.println(accelZ, 3);
}

void IMUController::printAllData() {
    printGyroData();
    printAccelData();
}

void IMUController::forceDefaultImuConfig() {
    // Accel: ODR=104 Hz (0100xxxx), FS=±4 g (FS_XL=10) -> 0x4A
    writeReg(REG_CTRL1_XL, 0x4A);
    // Gyro : ODR=104 Hz, FS=±2000 dps (FS_G=11), FS_125=0 -> 0x4C
    // (This matches Arduino_LSM6DS3's conversion to deg/s)
    writeReg(REG_CTRL2_G, 0x4C);
}

void IMUController::printImuConfig() {
    uint8_t a = readReg(REG_CTRL1_XL);
    uint8_t g = readReg(REG_CTRL2_G);

    auto odrToHz = [](uint8_t odr) -> int {
        switch (odr) {
            case 1: return 12;  case 2: return 26;  case 3: return 52;
            case 4: return 104; case 5: return 208; case 6: return 416;
            case 7: return 833; case 8: return 1666;
            default: return 0;
        }
    };

    uint8_t odr_xl = (a >> 4) & 0x0F;
    uint8_t fs_xl  = (a >> 2) & 0x03; // 00:±2g, 01:±16g, 10:±4g, 11:±8g
    uint8_t odr_g  = (g >> 4) & 0x0F;
    uint8_t fs_g   = (g >> 2) & 0x03; // 00:±250, 01:±500, 10:±1000, 11:±2000

    const char* fsXLstr[] = {"±2 g","±16 g","±4 g","±8 g"};
    const char* fsGstr[]  = {"±250 dps","±500 dps","±1000 dps","±2000 dps"};

    Serial.print("[IMU] Accel: ODR=");
    Serial.print(odrToHz(odr_xl)); Serial.print(" Hz, FS=");
    Serial.println(fsXLstr[fs_xl]);

    Serial.print("[IMU] Gyro : ODR=");
    Serial.print(odrToHz(odr_g)); Serial.print(" Hz, FS=");
    Serial.println(fsGstr[fs_g]);
}
