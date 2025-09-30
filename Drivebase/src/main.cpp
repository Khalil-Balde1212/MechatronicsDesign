#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MOTOR_FLA 7
#define MOTOR_FLB 6
#define MOTOR_FRA 4
#define MOTOR_FRB 5
#define MOTOR_BLA 8
#define MOTOR_BLB 9
#define MOTOR_BRA 10
#define MOTOR_BRB 11

//2, 3, 9, 10, 11
#define ENC_FLA 10
#define ENC_FLB 9
#define ENC_FRA 11
#define ENC_FRB 8
#define ENC_BLA 2
#define ENC_BLB 5
#define ENC_BRA 3
#define ENC_BRB 4 

int cpr = 1440; // Counts per revolution

// PID constants for front left motor only
float kp = 10.0;
float ki = 0.0;
float kd = 0.0;

// PID variables for front left motor
long setpoint_fl = 0;
float integral_fl = 0;
float prev_error_fl = 0;
unsigned long last_time = 0;

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
void setPositionFL(float rotations);
void setPositionDegreesFL(float degrees);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  
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

  setSpeeds(0,0);
  last_time = millis();
  
  Serial.println("Front Left PID Position Control Initialized");
  Serial.println("Commands:");
  Serial.println("- Send 'r1.5' to rotate FL motor 1.5 rotations");
  Serial.println("- Send 'd90' to rotate FL motor 90 degrees");
  Serial.println("- Send 'reset' to reset FL position");
}

void loop() {
  // Update PID control for front left motor
  float pidOutput = pidControlFL();
  setMotorSpeed(MOTOR_FLA, MOTOR_FLB, (int)pidOutput);
  
  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("r")) {
      // Rotation command: r1.5 = 1.5 rotations
      float rotations = command.substring(1).toFloat();
      setPositionFL(rotations);
      Serial.print("Setting FL motor to ");
      Serial.print(rotations);
      Serial.println(" rotations");
        }
        else if (command.startsWith("kp")) {
      kp = command.substring(2).toFloat();
      Serial.print("Updated kp to: ");
      Serial.println(kp);
        }
        else if (command.startsWith("ki")) {
      ki = command.substring(2).toFloat();
      Serial.print("Updated ki to: ");
      Serial.println(ki);
        }
        else if (command.startsWith("kd")) {
      kd = command.substring(2).toFloat();
      Serial.print("Updated kd to: ");
      Serial.println(kd);
        }
        else if (command == "reset") {
      noInterrupts();
      encoderCountFL = 0;
      interrupts();
      setPositionFL(0);
      Serial.println("FL position reset");
    }
    else if (command == "status") {
      Serial.print("FL Current: ");
      Serial.print(encoderCountFL / (float)cpr, 2);
      Serial.print(" rotations, Setpoint: ");
      Serial.print(setpoint_fl / (float)cpr, 2);
      Serial.println(" rotations");
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
    encoderCountFL++;  // Forward direction
  } else {
    encoderCountFL--;  // Reverse direction
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
    encoderCountBL++;
  } else {
    encoderCountBL--;
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
  float dt = (now - last_time) / 1000.0; // Convert to seconds
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

  last_time = now;
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