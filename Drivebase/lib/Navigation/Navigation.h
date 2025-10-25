#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "MotorController.h"
#include "tof_cameras.h"
#include <Arduino_LSM9DS1.h>

namespace Navigation {

class Navigation {
public:
    Navigation(MotorController& motors, TOF::TOFSensors& sensors);
    
    void begin();
    void update();
    
    // Basic movement commands
    void moveForward(float rotations);
    void moveBackward(float rotations);
    void turnLeft(float rotations);
    void turnRight(float rotations);
    void stop();
    
    // Sensor access
    float getDistance(int sensorIndex);
    float getMinDistance();
    bool isObstacleAhead(float thresholdCM = 30.0f);
    
    // Status
    bool isMoving();
    
    // Auto navigation
    void startAuto();
    void stopAuto();
    bool isAutoActive();
    
private:
    MotorController& motorController;
    TOF::TOFSensors& tofSensors;
    
    void updateSensors();
    void updateMotorStatus();
    void updateAutoNavigation();
    void updateIMU();
    float calculateCenteringPID();
    void captureInitialReadings();
    bool hasExitedTunnel();
    
    bool moving;
    float minDistance;
    bool autoActive;
    float initialHeading;
    float currentHeading;
    bool imuAvailable;
    
    // Wall follow PID
    float wallTargetDistance;
    float wallPID_Kp;
    float wallPID_Ki;
    float wallPID_Kd;
    float wallPID_integral;
    float wallPID_lastError;
    unsigned long wallPID_lastTime;
    
    // Robot dimensions (mm)
    static constexpr float ROBOT_WIDTH = 146.0f;
    static constexpr float ROBOT_LENGTH = 254.0f;
    
    // Tunnel detection
    float initialSensorReadings[4];
    bool tunnelDetected;
    bool exitedTunnel;
};

} // namespace Navigation

#endif