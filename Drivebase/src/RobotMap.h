#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include <vector>

namespace RobotMap
{
    // Tank Drive Configuration
    // Left Side Motors
    const int ENC_LEFT_A = 2;
        const int ENC_LEFT_B = 7;
    const boolean ENC_LEFT_INVERTED = true;
    const int ENC_RIGHT_CPR = 750;

    const int MOTOR_LEFT_A = 13;
    const int MOTOR_LEFT_B = 12;
    const boolean MOTOR_LEFT_INVERTED = true;

    // Right Side Motors
    const int ENC_RIGHT_A = 10;
    const int ENC_RIGHT_B = 9;
    const boolean ENC_RIGHT_INVERTED = true;
    const int ENC_LEFT_CPR = 750;
    
    const int MOTOR_RIGHT_A = 7;
    const int MOTOR_RIGHT_B = 6;
    const boolean MOTOR_RIGHT_INVERTED = false;

    // Pivot Motors (for turning)
    // Pivot Left Motors
    const int ENC_PIVOT_LEFT_A = 11;
    const int ENC_PIVOT_LEFT_B = 8;
    const boolean ENC_PIVOT_LEFT_INVERTED = true;
    const int ENC_PIVOT_LEFT_CPR = 2890;

    const int MOTOR_PIVOT_LEFT_A = 15; 
    const int MOTOR_PIVOT_LEFT_B = 14;  
    const boolean MOTOR_PIVOT_LEFT_INVERTED = false;

    // Pivot Right Motors
    const int ENC_PIVOT_RIGHT_A = 3;
    const int ENC_PIVOT_RIGHT_B = 4;
    const boolean ENC_PIVOT_RIGHT_INVERTED = false;  
    const int ENC_PIVOT_RIGHT_CPR = 1160;

    const int MOTOR_PIVOT_RIGHT_A = 5; 
    const int MOTOR_PIVOT_RIGHT_B = 4;  
    const boolean MOTOR_PIVOT_RIGHT_INVERTED = false;



    //robot geometry
    const std::vector<float> FL_WHEEL_POS = {38.1, 38.1};  // Front Left
    const std::vector<float> FR_WHEEL_POS = {38.1, -38.1}; // Front Right
    const std::vector<float> RL_WHEEL_POS = {-38.1, 38.1}; // Rear Left
    const std::vector<float> RR_WHEEL_POS = {-38.1, -38.1}; // Rear Right

    const float WHEEL_RADIUS_MM = 12.7; // in mm

    const float MAXIMUM_WHEEL_RPS = 2.0;
    const float MAXIMUM_VELOCITY = MAXIMUM_WHEEL_RPS * WHEEL_RADIUS_MM * 2 * PI / 1000.0; // in m/s
}
#endif