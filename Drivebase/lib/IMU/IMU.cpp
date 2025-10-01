#include "IMU.h"

IMUController::IMUController() {
    gyroX = gyroY = gyroZ = 0.0;
    accelX = accelY = accelZ = 0.0;
    initialized = false;
}

bool IMUController::begin() {
    if (IMU.begin()) {
        initialized = true;
        Serial.println("IMU initialized successfully.");
        return true;
    } else {
        Serial.println("Failed to initialize IMU!");
        initialized = false;
        return false;
    }
}

void IMUController::update() {
    if (!initialized) return;
    
    readGyroscope();
    readAccelerometer();
}

bool IMUController::readGyroscope() {
    if (!initialized) return false;
    
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        return true;
    }
    return false;
}

bool IMUController::readAccelerometer() {
    if (!initialized) return false;
    
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
        return true;
    }
    return false;
}

void IMUController::printGyroData() {
    Serial.print("Gyro: X=");
    Serial.print(gyroX);
    Serial.print("\tY=");
    Serial.print(gyroY);
    Serial.print("\tZ=");
    Serial.println(gyroZ);
}

void IMUController::printAccelData() {
    Serial.print("Accel: X=");
    Serial.print(accelX);
    Serial.print("\tY=");
    Serial.print(accelY);
    Serial.print("\tZ=");
    Serial.println(accelZ);
}

void IMUController::printAllData() {
    printGyroData();
    printAccelData();
}