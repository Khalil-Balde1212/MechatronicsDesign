#include <Arduino.h>
#define MOTOR_FLA 27
#define MOTOR_FLB 14
#define MOTOR_FRA 32
#define MOTOR_FRB 33
#define MOTOR_BLA 25
#define MOTOR_BLB 26
#define MOTOR_BRA 22
#define MOTOR_BRB 23

void setSpeeds(float left, float right);
void setMotorSpeed(int motorPin1, int motorPin2, int speed);

void setup() {
  Serial.begin(9600);
}

void loop() {
  setSpeeds(1, 1);
  delay(1000);
  setSpeeds(-1, -1);
  delay(1000);
  setSpeeds(0, 0);
  delay(1000);
  setSpeeds(1, -1);
  delay(1000);
  setSpeeds(-1, 1);
  delay(1000);
  setSpeeds(0, 0);
  delay(2000);
}

/**
 * Sets the speeds of the left and right motors.
 * 
 * @param motorPin1 The pin for first motor hbridge pin.
 * @param motorPin2 The pin for second motor hbridge pin.
 * @param speed The speed of the motor (-255 to 255).
 */
void setMotorSpeed(int motorPin1, int motorPin2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(motorPin1, speed);
    analogWrite(motorPin2, 0);
  } else if (speed < 0) {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, -speed);
  } else {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
  }
}

void setSpeeds(float left, float right) {
  left = left * 255;
  right = right * 255;
  setMotorSpeed(MOTOR_FLA, MOTOR_FLB, left);
  setMotorSpeed(MOTOR_BLA, MOTOR_BLB, left);
  setMotorSpeed(MOTOR_FRA, MOTOR_FRB, right);
  setMotorSpeed(MOTOR_BRA, MOTOR_BRB, right);
}

