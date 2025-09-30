#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MOTOR_FLA 4
#define MOTOR_FLB 5
#define MOTOR_FRA 6
#define MOTOR_FRB 7
#define MOTOR_BLA 8
#define MOTOR_BLB 9
#define MOTOR_BRA 10
#define MOTOR_BRB 11

void setSpeeds(float left, float right);
void setMotorSpeed(int motorPin1, int motorPin2, int speed);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
}

void loop() {
  digitalWrite(23, !digitalRead(23));
  //sweep all dirs
  for (float speed = 0; speed <= 1; speed += 0.05) {
    setSpeeds(speed, speed);
    delay(100);
  }
  delay(500);
  for (float speed = 1; speed >= 0; speed -= 0.05) {
    setSpeeds(speed, speed);
    delay(100);
  }
  delay(500);

  for (float speed = 0; speed <= 1; speed += 0.05) {
    setSpeeds(-speed, -speed);
    delay(100);
  }
  delay(500);
  for (float speed = 1; speed >= 0; speed -= 0.05) {
    setSpeeds(--speed, -speed);
    delay(100);
  }
  delay(500);

  for (float speed = 0; speed <= 1; speed += 0.05) {
    setSpeeds(speed, -speed);
    delay(100);
  }
  delay(500);
  for (float speed = 1; speed >= 0; speed -= 0.05) {
    setSpeeds(speed, -speed);
    delay(100);
  }
  delay(500);

  for (float speed = 0; speed <= 1; speed += 0.05) {
    setSpeeds(-speed, speed);
    delay(100);
  }
  delay(500);
  for (float speed = 1; speed >= 0; speed -= 0.05) {
    setSpeeds(-speed, speed);
    delay(100);
  }
  delay(500);
}

/**
 * Sets the speeds of the left and right motors.
 * 
 * @param motorPin1 The pin for first motor hbridge pin.
 * @param motorPin2 The pin for second motor hbridge pin.
 * @param speed The speed of the motor (-255 to 255).
 */
void setMotorSpeed(int motorPin1, int motorPin2, int speed) {
  speed = constrain(speed, -4095, 4095);

  if (speed > 0) {
    pwm.setPWM(motorPin1, 0, speed);
    pwm.setPWM(motorPin2, 0, 0);
  } else if (speed < 0) {
    pwm.setPWM(motorPin1, 0, 0);
    pwm.setPWM(motorPin2, 0, -speed);
  } else {
    pwm.setPWM(motorPin1, 0, 0);
    pwm.setPWM(motorPin2, 0, 0);
  }
}

void setSpeeds(float left, float right) {
  left = left * 4096;
  right = right * 4906;
  setMotorSpeed(MOTOR_FLA, MOTOR_FLB, left);
  setMotorSpeed(MOTOR_BLA, MOTOR_BLB, left);
  setMotorSpeed(MOTOR_FRA, MOTOR_FRB, right);
  setMotorSpeed(MOTOR_BRA, MOTOR_BRB, right);
}

