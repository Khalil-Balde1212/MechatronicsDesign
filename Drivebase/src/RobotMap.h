#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include <vector>

namespace RobotMap
{
    // Tank Drive Configuration
    // Left Side Motors
    const int ENC_LEFT_A = 2;
    const int ENC_LEFT_B = 7;
    const int MOTOR_LEFT_A = 5;   // Swapped from 12
    const int MOTOR_LEFT_B = 4;   // Swapped from 13

    // Right Side Motors
    const int ENC_RIGHT_A = 10;
    const int ENC_RIGHT_B = 9;
    const int MOTOR_RIGHT_A = 12;  // Swapped from 5
    const int MOTOR_RIGHT_B = 13;  // Swapped from 4

    // Pivot Motors (for turning)
    const int ENC_PIVOT_FRONT_A = 3;
    const int ENC_PIVOT_FRONT_B = 4;
    const int MOTOR_PIVOT_FRONT_A = 14;  // Swapped back from 7
    const int MOTOR_PIVOT_FRONT_B = 15;  // Swapped back from 6

    const int ENC_PIVOT_REAR_A = 11;
    const int ENC_PIVOT_REAR_B = 8;
    const int MOTOR_PIVOT_REAR_A = 7;   // Swapped from 14
    const int MOTOR_PIVOT_REAR_B = 6;   // Swapped from 15

    const std::vector<float> FL_WHEEL_POS = {38.1, 38.1};  // Front Left
    const std::vector<float> FR_WHEEL_POS = {38.1, -38.1}; // Front Right
    const std::vector<float> RL_WHEEL_POS = {-38.1, 38.1}; // Rear Left
    const std::vector<float> RR_WHEEL_POS = {-38.1, -38.1}; // Rear Right

    const float WHEEL_RADIUS_MM = 12.7; // in mm
}
#endif