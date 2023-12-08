#include <Wire.h>

#define MOTOR_A1    9
#define MOTOR_A2    10
#define MOTOR_B1    11
#define MOTOR_B2    12

#define LINE_SENSOR_1 A2
#define LINE_SENSOR_2 A3
#define LINE_SENSOR_3 A4
#define LINE_SENSOR_4 A5
#define LINE_SENSOR_5 A6

#define ENCODER_A_A  21
#define ENCODER_A_B  20
#define ENCODER_B_A  18
#define ENCODER_B_B  19

// PID constants
#define KP  0.1   // Proportional gain
#define KI  0.01  // Integral gain
#define KD  0.1   // Derivative gain

// Target position for the line sensor
#define TARGET_POSITION 512  // Adjust as needed

int previousError = 0;
int integral = 0;
int consecutiveOffLineCount = 0;
const int MAX_CONSECUTIVE_OFFLINE_COUNT = 5;  // Adjust as needed

volatile int encoderCountA = 0;
volatile int encoderCountB = 0;

void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  pinMode(ENCODER_A_A, INPUT);
  pinMode(ENCODER_A_B, INPUT);
  pinMode(ENCODER_B_A, INPUT);
  pinMode(ENCODER_B_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_B), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_A), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_B), updateEncoderB, CHANGE);

  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // Read line sensor values
  int sensor1 = analogRead(LINE_SENSOR_1);
  int sensor2 = analogRead(LINE_SENSOR_2);
  int sensor3 = analogRead(LINE_SENSOR_3);
  int sensor4 = analogRead(LINE_SENSOR_4);
  int sensor5 = analogRead(LINE_SENSOR_5);

  // Calculate error for PID control
  int currentPosition = (sensor1 + sensor2 + sensor3 + sensor4 + sensor5) / 5;
  int error = TARGET_POSITION - currentPosition;

  // Calculate PID components
  int proportional = KP * error;
  integral += KI * error;
  int derivative = KD * (error - previousError);

  // Calculate PID output
  int pidOutput = proportional + integral + derivative;

  // Map PID output to motor speeds
  int motorSpeedA = constrain(255 - pidOutput, 0, 255);
  int motorSpeedB = constrain(255 + pidOutput, 0, 255);

  // Set motor speeds
  analogWrite(MOTOR_A1, motorSpeedA);
  analogWrite(MOTOR_A2, LOW);
  analogWrite(MOTOR_B1, motorSpeedB);
  analogWrite(MOTOR_B2, LOW);

  // Update previous error for the next iteration
  previousError = error;

  // Read data from ROS system
  if (Serial.available() > 0) {
    char command = Serial.read();

    // Process the command
    if (command == 'y') {
      // Obstacle detected, stop the motors
      stopMotors();
      delay(500);
      turnRight();
    } else if (command == 'n') {
      // No obstacle, continue normal operation
      // Line-following logic already handled by PID controller
      if (isOffLine()) {
        consecutiveOffLineCount++;
        if (consecutiveOffLineCount >= MAX_CONSECUTIVE_OFFLINE_COUNT) {
          // Too far to the right, initiate a recovery maneuver
          recoveryManeuver();
          consecutiveOffLineCount = 0;
        }
      } else {
        consecutiveOffLineCount = 0;
      }
    }
  }

  // Your main loop code here
}

void updateEncoderA() {
  if (digitalRead(ENCODER_A_A) == digitalRead(ENCODER_A_B)) {
    encoderCountA++;
  } else {
    encoderCountA--;
  }
}

void updateEncoderB() {
  if (digitalRead(ENCODER_B_A) == digitalRead(ENCODER_B_B)) {
    encoderCountB++;
  } else {
    encoderCountB--;
  }
}

void stopMotors() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

void turnRight() {
  analogWrite(MOTOR_A1, 255);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 255);
}

bool isOffLine() {
  // Implement your logic to determine if the robot is off the line
  // For example, check if all line sensors are reading low values
  return analogRead(LINE_SENSOR_1) < 100 && analogRead(LINE_SENSOR_5) < 100;
}

void recoveryManeuver() {
  // Implement a recovery maneuver to bring the robot back to the line
  // For example, stop the motors, turn left, and then move forward
  stopMotors();
  delay(500);
  turnLeft();
  delay(500);
  moveForward();
}

void turnLeft() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 255);
  analogWrite(MOTOR_B1, 255);
  analogWrite(MOTOR_B2, 0);
}

void moveForward() {
  analogWrite(MOTOR_A1, 255);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 255);
  analogWrite(MOTOR_B2, 0);
}