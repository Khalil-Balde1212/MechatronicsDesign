#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MOTOR_FLA 14
#define MOTOR_FLB 15
#define MOTOR_FRA 5
#define MOTOR_FRB 4
#define MOTOR_BLA 13
#define MOTOR_BLB 12
#define MOTOR_BRA 7
#define MOTOR_BRB 6

//2, 3, 9, 10, 11 ISRs
#define ENC_FLA 3
#define ENC_FLB 4
#define ENC_FRA 11
#define ENC_FRB 8
#define ENC_BLA 10
#define ENC_BLB 9
#define ENC_BRA 2
#define ENC_BRB 7 

int cpr = 1440; // Counts per revolution

// PID constants for front left motor only
float kp = 15.0;
float ki = 0.0;
float kd = 0.0;

// PID variables for front left motor
long setpoint_fl, setpoint_fr, setpoint_bl, setpoint_br = 0;
float integral_fl, integral_fr, integral_bl, integral_br = 0;
float prev_error_fl, prev_error_fr, prev_error_bl, prev_error_br = 0;
unsigned long last_time_fl, last_time_fr, last_time_bl, last_time_br = 0;

// Encoder variables
volatile long encoderCountFL = 0;
volatile long encoderCountFR = 0;
volatile long encoderCountBL = 0;
volatile long encoderCountBR = 0;

// Encoder interrupt service routines
void encoderISR_FL();
void encoderISR_FR();
void encoderISR_BL();
void encoderISR_BR();

void setSpeeds(float left, float right);
void setMotorSpeed(int motorPin1, int motorPin2, int speed);
void resetEncoders();
void printEncoderCounts();
void printRotations();
float pidControlFL();
float pidControlFR();
float pidControlBL();
float pidControlBR();
void setPositionFL(float rotations);
void setPositionFR(float rotations);
void setPositionBL(float rotations);
void setPositionBR(float rotations);
void setPositionDegreesFL(float degrees);
void setPositionDegreesFR(float degrees);
void setPositionDegreesBL(float degrees);
void setPositionDegreesBR(float degrees);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(800);  // This is the maximum PWM frequency
  
  // Initialize encoder pins as inputs with pull-up resistors
  pinMode(ENC_FLA, INPUT_PULLUP);
  pinMode(ENC_FLB, INPUT_PULLUP);
  pinMode(ENC_FRA, INPUT_PULLUP);
  pinMode(ENC_FRB, INPUT_PULLUP);
  pinMode(ENC_BLA, INPUT_PULLUP);
  pinMode(ENC_BLB, INPUT_PULLUP);
  pinMode(ENC_BRA, INPUT_PULLUP);
  pinMode(ENC_BRB, INPUT_PULLUP);
  
  // Check which pins support interrupts
  Serial.println(digitalPinToInterrupt(ENC_FLA));
  Serial.println(digitalPinToInterrupt(ENC_FRA));
  Serial.println(digitalPinToInterrupt(ENC_BLA));
  Serial.println(digitalPinToInterrupt(ENC_BRA));
  
  attachInterrupt(digitalPinToInterrupt(ENC_FLA), encoderISR_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FRA), encoderISR_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BLA), encoderISR_BL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BRA), encoderISR_BR, RISING);

  setSpeeds(0, 0);
  last_time_fl = last_time_fr = last_time_bl = last_time_br = millis();
  
  Serial.println("Front Left PID Position Control Initialized");
  Serial.println("Commands:");
  Serial.println("- Send 'r1.5' to rotate FL motor 1.5 rotations");
  Serial.println("- Send 'd90' to rotate FL motor 90 degrees");
  Serial.println("- Send 'reset' to reset FL position");
  delay(1000);
}

void loop() {
  // Update PID control for front left motor
  float pidOutputFL = pidControlFL();
  setMotorSpeed(MOTOR_FLA, MOTOR_FLB, (int)pidOutputFL);

  float pidOutputFR = pidControlFR();
  setMotorSpeed(MOTOR_FRA, MOTOR_FRB, (int)pidOutputFR);

  float pidOutputBL = pidControlBL();
  setMotorSpeed(MOTOR_BLA, MOTOR_BLB, (int)pidOutputBL);

  float pidOutputBR = pidControlBR();
  setMotorSpeed(MOTOR_BRA, MOTOR_BRB, (int)pidOutputBR);
  
  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    switch (command[0]) {
      // rotate command rX####
      // X = side: r=right, l=left, a=front right, b=back right, c=front left, d=back left,
      // #### = number of rotations (float)
      case 'r': {
        char side = command[1];
        float rotations = command.substring(2).toFloat();

        switch(side){
          case 'r':
            setPositionFR(rotations);
            setPositionBR(rotations);
            break;

          case 'l':
            setPositionFL(rotations);
            setPositionBL(rotations);
            break;

          case 'a':
            setPositionFR(rotations);
            break;

          case 'b':
            setPositionBR(rotations);
            break;

          case 'c':
            setPositionFL(rotations);
            break;

          case 'd':
            setPositionBL(rotations);
            break;

          case 'f':
            setPositionFR((encoderCountFR / (float)cpr) + rotations);
            setPositionFL((encoderCountFL / (float)cpr) + rotations);
            setPositionBR((encoderCountBR / (float)cpr) + rotations);
            setPositionBL((encoderCountBL / (float)cpr) + rotations);
            break;
          case 't':
            setPositionFR((encoderCountFR / (float)cpr) + rotations);
            setPositionFL((encoderCountFL / (float)cpr) - rotations);
            setPositionBR((encoderCountBR / (float)cpr) + rotations);
            setPositionBL((encoderCountBL / (float)cpr) - rotations);
            break;
        }
        break;
      }
      
      // set pid gains command kX####
      // X = gain type: p=kp, i=ki, d=kd
      // #### = new gain value (float)
      case 'k': {
        if (command.startsWith("kp")) {
          kp = command.substring(2).toFloat();
          Serial.print("Updated kp to: ");
          Serial.println(kp);
        } else if (command.startsWith("ki")) {
          ki = command.substring(2).toFloat();
          Serial.print("Updated ki to: ");
          Serial.println(ki);
        } else if (command.startsWith("kd")) {
          kd = command.substring(2).toFloat();
          Serial.print("Updated kd to: ");
          Serial.println(kd);
        }
      }
      break;

      case 'x': {
        encoderCountFL = 0;
        encoderCountFR = 0;
        encoderCountBL = 0;
        encoderCountBR = 0;
        setpoint_fl = 0;
        setpoint_fr = 0;
        setpoint_bl = 0;
        setpoint_br = 0;
        break;
      }
      
      case 's': {
        if (command == "status") {
          Serial.print("FL Current: ");
          Serial.print(encoderCountFL / (float)cpr, 2);
          Serial.print(" rotations, Setpoint: ");
          Serial.print(setpoint_fl / (float)cpr, 2);
          Serial.println(" rotations");
        }
        break;
      }

      default: {
        Serial.println("Unknown command. Available commands:");
        Serial.println("- rX####: Rotate motors (X: side, ####: rotations)");
        Serial.println("  Sides: r=right, l=left, a=front right, b=back right, c=front left, d=back left");
        Serial.println("- kX####: Set PID gains (X: p=kp, i=ki, d=kd, ####: value)");
        Serial.println("- reset: Reset FL position");
        Serial.println("- status: Display FL current position and setpoint");
        break;
      }
    }
  }
  
  // Print status every 500ms
  static unsigned long last_print = 0;
  if (millis() - last_print > 500) {
    printRotations();
    last_print = millis();
  }
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
  if (speed != 0 && abs(speed) < 1500) {
    speed = (speed > 0) ? 1500 : -1500;
  }

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

// Encoder interrupt service routines
void encoderISR_FL() {
  // Read the state of both encoder pins
  bool stateA = digitalRead(ENC_FLA);
  bool stateB = digitalRead(ENC_FLB);
  
  // Determine direction based on A and B states
  if (stateA == stateB) {
    encoderCountFL--;  // Forward direction
  } else {
    encoderCountFL++;  // Reverse direction
  }
}

void encoderISR_FR() {
  bool stateA = digitalRead(ENC_FRA);
  bool stateB = digitalRead(ENC_FRB);
  
  if (stateA == stateB) {
    encoderCountFR++;
  } else {
    encoderCountFR--;
  }
}

void encoderISR_BL() {
  bool stateA = digitalRead(ENC_BLA);
  bool stateB = digitalRead(ENC_BLB);
  
  if (stateA == stateB) {
    encoderCountBL--;
  } else {
    encoderCountBL++;
  }
}

void encoderISR_BR() {
  bool stateA = digitalRead(ENC_BRA);
  bool stateB = digitalRead(ENC_BRB);
  
  if (stateA == stateB) {
    encoderCountBR++;
  } else {
    encoderCountBR--;
  }
}

// Reset all encoder counts to zero
void resetEncoders() {
  noInterrupts();  // Disable interrupts temporarily
  encoderCountFL = 0;
  encoderCountFR = 0;
  encoderCountBL = 0;
  encoderCountBR = 0;
  interrupts();    // Re-enable interrupts
}

// Print current encoder counts
void printEncoderCounts() {
  noInterrupts();  // Disable interrupts to get atomic read
  long countFL = encoderCountFL;
  long countFR = encoderCountFR;
  long countBL = encoderCountBL;
  long countBR = encoderCountBR;
  interrupts();    // Re-enable interrupts
  
  Serial.print("Encoders - FL: ");
  Serial.print(countFL);
  Serial.print(", FR: ");
  Serial.print(countFR);
  Serial.print(", BL: ");
  Serial.print(countBL);
  Serial.print(", BR: ");
  Serial.println(countBR);
}

// Print the number of rotations for each encoder
void printRotations() {
  noInterrupts();  // Disable interrupts to get atomic read
  float rotationsFL = encoderCountFL / (float)cpr;
  float rotationsFR = encoderCountFR / (float)cpr;
  float rotationsBL = encoderCountBL / (float)cpr;
  float rotationsBR = encoderCountBR / (float)cpr;
  interrupts();    // Re-enable interrupts
  
  Serial.print("Rotations - FL: ");
  Serial.print(rotationsFL, 2);
  Serial.print(", FR: ");
  Serial.print(rotationsFR, 2);
  Serial.print(", BL: ");
  Serial.print(rotationsBL, 2);
  Serial.print(", BR: ");
  Serial.println(rotationsBR, 2);
}

// PID control function for front left motor
float pidControlFL() {
  noInterrupts();
  unsigned long now = millis();
  float dt = (now - last_time_fl) / 1000.0; // Convert to seconds
  float error = setpoint_fl - encoderCountFL;
  interrupts();
  // Proportional term
  float P = kp * error;
  
  // Integral term
  integral_fl += error * dt;
  float I = ki * integral_fl;
  
  // Derivative term
  float D = kd * (error - prev_error_fl) / dt;
  prev_error_fl = error;
  
  // Combine PID terms
  float output = P + I + D;

  last_time_fl = now;
  return output;
}

float pidControlFR() {
  noInterrupts();
  unsigned long now = millis();
  float dt = (now - last_time_fr) / 1000.0; // Convert to seconds
  float error = setpoint_fr - encoderCountFR;
  interrupts();
  // Proportional term
  float P = kp * error;
  
  // Integral term
  integral_fr += error * dt;
  float I = ki * integral_fr;
  
  // Derivative term
  float D = kd * (error - prev_error_fr) / dt;
  prev_error_fr = error;
  
  // Combine PID terms
  float output = P + I + D;

  last_time_fr = now;
  return output;
}

float pidControlBL() {
  noInterrupts();
  unsigned long now = millis();
  float dt = (now - last_time_bl) / 1000.0; // Convert to seconds
  float error = setpoint_bl - encoderCountBL;
  interrupts();
  // Proportional term
  float P = kp * error;
  
  // Integral term
  integral_bl += error * dt;
  float I = ki * integral_bl;
  
  // Derivative term
  float D = kd * (error - prev_error_bl) / dt;
  prev_error_bl = error;
  
  // Combine PID terms
  float output = P + I + D;

  last_time_bl = now;
  return output;
}

float pidControlBR() {
  noInterrupts();
  unsigned long now = millis();
  float dt = (now - last_time_br) / 1000.0; // Convert to seconds
  float error = setpoint_br - encoderCountBR;
  interrupts();
  // Proportional term
  float P = kp * error;
  
  // Integral term
  integral_br += error * dt;
  float I = ki * integral_br;
  
  // Derivative term
  float D = kd * (error - prev_error_br) / dt;
  prev_error_br = error;
  
  // Combine PID terms
  float output = P + I + D;

  last_time_br = now;
  return output;
}

// Set position in rotations for front left motor
void setPositionFL(float rotations) {
  setpoint_fl = (long)(rotations * cpr);
  
  // Reset integral term to avoid windup
  integral_fl = 0;
  prev_error_fl = 0;
  
  Serial.print("FL setpoint set to: ");
  Serial.print(rotations, 2);
  Serial.println(" rotations");
}

// Set position in degrees for front left motor
void setPositionDegreesFL(float degrees) {
  float rotations = degrees / 360.0;
  setPositionFL(rotations);
}

// Set position in rotations for front right motor
void setPositionFR(float rotations) {
  setpoint_fr = (long)(rotations * cpr);
  
  // Reset integral term to avoid windup
  integral_fr = 0;
  prev_error_fr = 0;
  
  Serial.print("FR setpoint set to: ");
  Serial.print(rotations, 2);
  Serial.println(" rotations");
}

// Set position in degrees for front right motor
void setPositionDegreesFR(float degrees) {
  float rotations = degrees / 360.0;
  setPositionFR(rotations);
}

// Set position in rotations for back left motor
void setPositionBL(float rotations) {
  setpoint_bl = (long)(rotations * cpr);
  
  // Reset integral term to avoid windup
  integral_bl = 0;
  prev_error_bl = 0;
  
  Serial.print("BL setpoint set to: ");
  Serial.print(rotations, 2);
  Serial.println(" rotations");
}

// Set position in degrees for back left motor
void setPositionDegreesBL(float degrees) {
  float rotations = degrees / 360.0;
  setPositionBL(rotations);
}

// Set position in rotations for back right motor
void setPositionBR(float rotations) {
  setpoint_br = (long)(rotations * cpr);
  
  // Reset integral term to avoid windup
  integral_br = 0;
  prev_error_br = 0;
  
  Serial.print("BR setpoint set to: ");
  Serial.print(rotations, 2);
  Serial.println(" rotations");
}

// Set position in degrees for back right motor
void setPositionDegreesBR(float degrees) {
  float rotations = degrees / 360.0;
  setPositionBR(rotations);
}