#include "Navigation.h"
#include "Encoders.h"
#include <Arduino_LSM9DS1.h>

namespace Navigation {

Navigation::Navigation(MotorController& motors, TOF::TOFSensors& sensors)
    : motorController(motors), tofSensors(sensors), moving(false), minDistance(0.0f), 
      autoActive(false), initialHeading(0.0f), currentHeading(0.0f), imuAvailable(false),
      wallTargetDistance(11.0f), wallPID_Kp(0.03f), wallPID_Ki(0.0f), wallPID_Kd(0.005f),
      wallPID_integral(0.0f), wallPID_lastError(0.0f), wallPID_lastTime(0),
      tunnelDetected(false), exitedTunnel(false) {
    for (int i = 0; i < 4; i++) {
        initialSensorReadings[i] = -1.0f;
    }
}

void Navigation::begin() {
    moving = false;
    minDistance = 0.0f;
    autoActive = false;
    initialHeading = 0.0f;
    currentHeading = 0.0f;
    wallPID_integral = 0.0f;
    wallPID_lastError = 0.0f;
    wallPID_lastTime = 0;
    tunnelDetected = false;
    exitedTunnel = false;
    
    for (int i = 0; i < 4; i++) {
        initialSensorReadings[i] = -1.0f;
    }
    
    if (IMU.begin()) {
        imuAvailable = true;
    } else {
        imuAvailable = false;
    }
}

void Navigation::update() {
    updateSensors();
    updateMotorStatus();
    updateIMU();
    
    if (autoActive) {
        updateAutoNavigation();
    }
}

void Navigation::moveForward(float rotations) {
    float currentFL = motorController.getSetpointRotationsFL();
    float currentFR = motorController.getSetpointRotationsFR();
    float currentBL = motorController.getSetpointRotationsBL();
    float currentBR = motorController.getSetpointRotationsBR();
    
    motorController.setPositionFL(currentFL + rotations);
    motorController.setPositionFR(currentFR + rotations);
    motorController.setPositionBL(currentBL + rotations);
    motorController.setPositionBR(currentBR + rotations);
    
    moving = true;
}

void Navigation::moveBackward(float rotations) {
    float currentFL = motorController.getSetpointRotationsFL();
    float currentFR = motorController.getSetpointRotationsFR();
    float currentBL = motorController.getSetpointRotationsBL();
    float currentBR = motorController.getSetpointRotationsBR();
    
    motorController.setPositionFL(currentFL - rotations);
    motorController.setPositionFR(currentFR - rotations);
    motorController.setPositionBL(currentBL - rotations);
    motorController.setPositionBR(currentBR - rotations);
    
    moving = true;
}

void Navigation::turnLeft(float rotations) {
    float currentFL = motorController.getSetpointRotationsFL();
    float currentFR = motorController.getSetpointRotationsFR();
    float currentBL = motorController.getSetpointRotationsBL();
    float currentBR = motorController.getSetpointRotationsBR();
    
    motorController.setPositionFL(currentFL - rotations);
    motorController.setPositionFR(currentFR + rotations);
    motorController.setPositionBL(currentBL - rotations);
    motorController.setPositionBR(currentBR + rotations);
    
    moving = true;
}

void Navigation::turnRight(float rotations) {
    float currentFL = motorController.getSetpointRotationsFL();
    float currentFR = motorController.getSetpointRotationsFR();
    float currentBL = motorController.getSetpointRotationsBL();
    float currentBR = motorController.getSetpointRotationsBR();
    
    motorController.setPositionFL(currentFL + rotations);
    motorController.setPositionFR(currentFR - rotations);
    motorController.setPositionBL(currentBL + rotations);
    motorController.setPositionBR(currentBR - rotations);
    
    moving = true;
}

void Navigation::stop() {
    float currentFL = Encoders::getRotationsFL();
    float currentFR = Encoders::getRotationsFR();
    float currentBL = Encoders::getRotationsBL();
    float currentBR = Encoders::getRotationsBR();
    
    motorController.setPositionFL(currentFL);
    motorController.setPositionFR(currentFR);
    motorController.setPositionBL(currentBL);
    motorController.setPositionBR(currentBR);
    
    moving = false;
}

float Navigation::getDistance(int sensorIndex) {
    if (sensorIndex < 0 || sensorIndex >= TOF::SENSOR_COUNT) {
        return -1.0f;
    }
    return tofSensors.getFilteredDistanceCM(sensorIndex);
}

float Navigation::getMinDistance() {
    return minDistance;
}

bool Navigation::isObstacleAhead(float thresholdCM) {
    return minDistance > 0.0f && minDistance < thresholdCM;
}

bool Navigation::isMoving() {
    return moving;
}

void Navigation::startAuto() {
    autoActive = true;
    exitedTunnel = false;
    wallPID_integral = 0.0f;
    wallPID_lastError = 0.0f;
    wallPID_lastTime = millis();
    
    captureInitialReadings();
    
    if (imuAvailable) {
        updateIMU();
        initialHeading = currentHeading;
    }
}

void Navigation::stopAuto() {
    autoActive = false;
    stop();
}

bool Navigation::isAutoActive() {
    return autoActive;
}

void Navigation::updateSensors() {
    minDistance = -1.0f;
    
    for (int i = 0; i < TOF::SENSOR_COUNT; i++) {
        float distance = tofSensors.getFilteredDistanceCM(i);
        if (!tofSensors.sensorTimeout(i) && distance > 0.0f) {
            if (minDistance < 0.0f || distance < minDistance) {
                minDistance = distance;
            }
        }
    }
}

void Navigation::updateMotorStatus() {
    const float tolerance = 0.1f;
    
    float errorFL = abs(motorController.getSetpointRotationsFL() - Encoders::getRotationsFL());
    float errorFR = abs(motorController.getSetpointRotationsFR() - Encoders::getRotationsFR());
    float errorBL = abs(motorController.getSetpointRotationsBL() - Encoders::getRotationsBL());
    float errorBR = abs(motorController.getSetpointRotationsBR() - Encoders::getRotationsBR());
    
    if (errorFL < tolerance && errorFR < tolerance && errorBL < tolerance && errorBR < tolerance) {
        moving = false;
    }
}

void Navigation::updateAutoNavigation() {
    float sensor0 = tofSensors.getFilteredDistanceCM(0);
    float sensor1 = tofSensors.getFilteredDistanceCM(1);
    float sensor2 = tofSensors.getFilteredDistanceCM(2);
    float sensor3 = tofSensors.getFilteredDistanceCM(3);
    
    bool sensor0Valid = !tofSensors.sensorTimeout(0) && sensor0 > 0.0f;
    bool sensor1Valid = !tofSensors.sensorTimeout(1) && sensor1 > 0.0f;
    bool sensor2Valid = !tofSensors.sensorTimeout(2) && sensor2 > 0.0f;
    bool sensor3Valid = !tofSensors.sensorTimeout(3) && sensor3 > 0.0f;
    
    // Check if exited tunnel
    if (hasExitedTunnel()) {
        if (moving) {
            stop();
        }
        exitedTunnel = true;
        return;
    }
    
    // Stop if too close to front or back
    if ((sensor0Valid && sensor0 < 15.0f) || (sensor2Valid && sensor2 < 15.0f)) {
        if (moving) {
            stop();
        }
        return;
    }
    
    // Center the robot in the tunnel using left/right sensors
    if (!moving && sensor1Valid && sensor3Valid) {
        float centerError = abs(sensor3 - sensor1);
        
        // Deadband: only correct if error is more than 2cm
        if (centerError < 2.0f) {
            // Move straight when centered
            float baseRotations = 0.3f;
            
            float currentFL = motorController.getSetpointRotationsFL();
            float currentFR = motorController.getSetpointRotationsFR();
            float currentBL = motorController.getSetpointRotationsBL();
            float currentBR = motorController.getSetpointRotationsBR();
            
            motorController.setPositionFL(currentFL + baseRotations);
            motorController.setPositionFR(currentFR + baseRotations);
            motorController.setPositionBL(currentBL + baseRotations);
            motorController.setPositionBR(currentBR + baseRotations);
            
            moving = true;
        } else {
            // Apply correction
            float pidCorrection = calculateCenteringPID();
            
            float baseRotations = 0.3f;
            float leftRotations = baseRotations + pidCorrection;
            float rightRotations = baseRotations - pidCorrection;
            
            float currentFL = motorController.getSetpointRotationsFL();
            float currentFR = motorController.getSetpointRotationsFR();
            float currentBL = motorController.getSetpointRotationsBL();
            float currentBR = motorController.getSetpointRotationsBR();
            
            motorController.setPositionFL(currentFL + leftRotations);
            motorController.setPositionFR(currentFR + rightRotations);
            motorController.setPositionBL(currentBL + leftRotations);
            motorController.setPositionBR(currentBR + rightRotations);
            
            moving = true;
        }
    }
}

void Navigation::updateIMU() {
    if (!imuAvailable) return;
    
    if (IMU.gyroscopeAvailable()) {
        float gx, gy, gz;
        IMU.readGyroscope(gx, gy, gz);
        
        static unsigned long lastTime = millis();
        unsigned long currentTime = millis();
        float dt = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;
        
        currentHeading += gz * dt;
    }
}

float Navigation::calculateCenteringPID() {
    float sensor1 = tofSensors.getFilteredDistanceCM(1);
    float sensor3 = tofSensors.getFilteredDistanceCM(3);
    
    // Calculate centering error
    // Positive error = too close to left (sensor 1), need to turn right
    // Negative error = too close to right (sensor 3), need to turn left
    float centerError = sensor3 - sensor1;
    
    unsigned long currentTime = millis();
    float dt = (currentTime - wallPID_lastTime) / 1000.0f;
    wallPID_lastTime = currentTime;
    
    if (dt <= 0.0f || dt > 1.0f) {
        wallPID_lastError = centerError;
        return 0.0f;
    }
    
    // Disable integral term to prevent oscillation
    wallPID_integral = 0.0f;
    
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (centerError - wallPID_lastError) / dt;
    }
    wallPID_lastError = centerError;
    
    float output = wallPID_Kp * centerError + wallPID_Kd * derivative;
    
    return constrain(output, -0.1f, 0.1f);
}

void Navigation::captureInitialReadings() {
    for (int i = 0; i < 4; i++) {
        float distance = tofSensors.getFilteredDistanceCM(i);
        if (!tofSensors.sensorTimeout(i) && distance > 0.0f) {
            initialSensorReadings[i] = distance;
        } else {
            initialSensorReadings[i] = -1.0f;
        }
    }
    
    tunnelDetected = (initialSensorReadings[1] > 0.0f && initialSensorReadings[3] > 0.0f);
}

bool Navigation::hasExitedTunnel() {
    if (!tunnelDetected) return false;
    
    int sensorsExceeded = 0;
    
    for (int i = 0; i < 4; i++) {
        if (initialSensorReadings[i] > 0.0f) {
            float currentDistance = tofSensors.getFilteredDistanceCM(i);
            bool valid = !tofSensors.sensorTimeout(i) && currentDistance > 0.0f;
            
            if (valid && currentDistance > initialSensorReadings[i] + 20.0f) {
                sensorsExceeded++;
            }
        }
    }
    
    return sensorsExceeded >= 2;
}

} // namespace Navigation