#ifndef ROBOTMAP_H
#define ROBOTMAP_H

namespace RobotMap {
    // Tank Drive Configuration
    // Left Side Motors
    const int ENC_LEFT_A = 3;
    const int ENC_LEFT_B = 4;
    const int MOTOR_LEFT_A = 12;
    const int MOTOR_LEFT_B = 13;

    // Right Side Motors  
    const int ENC_RIGHT_A = 11;
    const int ENC_RIGHT_B = 8;
    const int MOTOR_RIGHT_A = 5;
    const int MOTOR_RIGHT_B = 6; 

    // Pivot Motors (for turning)
    const int ENC_PIVOT_FRONT_A = 10;
    const int ENC_PIVOT_FRONT_B = 9;
    const int MOTOR_PIVOT_FRONT_A = 14;
    const int MOTOR_PIVOT_FRONT_B = 15;
    
    const int ENC_PIVOT_REAR_A = 2;
    const int ENC_PIVOT_REAR_B = 7;
    const int MOTOR_PIVOT_REAR_A = 7;
    const int MOTOR_PIVOT_REAR_B = 8;


    
}

#endif