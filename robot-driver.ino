#include <QTRSensors.h>

#define MotFwdL 9   // Motor Forward pin left
#define MotRevL 10  // Motor Reverse pin left

#define MotFwdR 11  // Motor Forward pin right
#define MotRevR 12  // Motor Reverse pin right

int encoderPin1 = 20;  // int. 2
int encoderPin2 = 21;  // int. 3

int encoderPin3 = 18;  // int. 4
int encoderPin4 = 19;  // int. 5

int enL = 8;
int enR = 13;

int P;
int I;
int D;

float Kp = 0.05;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

void setup() {
  pinMode(MotFwdL, OUTPUT);
  pinMode(MotRevL, OUTPUT);
  pinMode(MotFwdR, OUTPUT);
  pinMode(MotRevR, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP);
  pinMode(encoderPin4, INPUT_PULLUP);
  digitalWrite(encoderPin1, HIGH);  // turn pullup resistor on
  digitalWrite(encoderPin2, HIGH);  // turn pullup resistor on
  digitalWrite(encoderPin3, HIGH);  // turn pullup resistor on
  digitalWrite(encoderPin4, HIGH);  // turn pullup resistor on
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A2, A3, A4, A5, A6 }, SensorCount);

  Serial.println("Calibrating...");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop() {
  PID_control();
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  Serial.print("Sensors: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print("Position: ");
  Serial.print(positionLine);
  Serial.print('\t');

  int error = 2000 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error;

  int motorSpeedChange = P * Kp + I * Ki + D * Kd;
  Serial.print("Error: ");
  Serial.print(motorSpeedChange);
  Serial.print("\t");

  int motorSpeedA = 200 - motorSpeedChange;
  int motorSpeedB = 200 + motorSpeedChange;
  Serial.print("Speed Left:");
  Serial.print(motorSpeedA);
  Serial.print(" ");
  Serial.print("Speed Right:");
  Serial.println(motorSpeedB);

  if (motorSpeedA > 255) {
    motorSpeedA = 255;
  }
  if (motorSpeedB > 255) {
    motorSpeedB = 255;
  }
  if (motorSpeedA < 0) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 0) {
    motorSpeedB = 0;
  }
  forward_movement(motorSpeedA, motorSpeedB);
}

void forward_movement(int speedA, int speedB) {
  digitalWrite(MotFwdL, HIGH);
  digitalWrite(MotRevL, LOW);
  digitalWrite(MotFwdR, HIGH);
  digitalWrite(MotRevR, LOW);
  analogWrite(enL, speedA);
  analogWrite(enR, speedB);
}