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

const int speed = 200;
const int delayTime = 300;
const int delayTurn = 180;
const int delayForward = 200;

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


void action(String sensorValue)
{
  const int value = sensorValue.toInt();
  switch (value)
  {
  case 0: // 0000
    Serial.println("Go forward!");
    goForward(speed);
    delay(delayForward);
    break;

  case 1: // 0001
    Serial.println("Detected wall in front at a right angle. Turn left!");
    turnLeft(speed);
    delay(delayTime);
    break;

  case 10: // 0010
    Serial.println("Detected wall in front at a left angle. Go back, then turn right!");
    goBackward(speed);
    // delay(delayTime);
    delay(150);
    stop();
    delay(1000);
    turnRight(speed, 45);
    delay(delayTime);
    break;

  case 11: // 0011
    Serial.println("Dectected wall in front with both side wall open. Turn left!");
    goBackward(speed);
    // delay(delayTime);
    delay(150);
    stop();
    delay(1000);
    turnLeft(speed);
    delay(delayTime);
    break;

  case 100: // 0100
    Serial.println("Dectected left wall open. Turn left!");
    turnLeft(speed, 50);
    delay(delayTime*2/3);
    stop();
    delay(1000);
    turnLeft(speed, 50);
    delay(delayTime*8/9);
    break;

  case 101: // 0101
    Serial.println("Dectected too close to right wall. Turn left 45 degrees!");
    turnLeft(speed, 45);
    break;

  case 110: // 0110
    Serial.println("Detected wall in front at a right angle but too close to right wall. Turn left 135 degrees!");
    goBackward(speed);
    delay(100);
    stop();
    delay(1000);
    turnLeft(speed, 90);
    delay(delayTime);
    break;

  case 111: // 0111
    Serial.println("Detected wall in front too close and right wall. Go back a bit then turn left!");
    goBackward(speed);
    delay(180);
    break;

  case 1000:
    Serial.println("Detected left wall close and in front open. Go forward!");
    goForward(speed);
    delay(delayForward);
    break;

  case 1001:
    Serial.println("Detected wall in front at a left angle but too close to left wall. Turn left 45 degrees!");
    turnLeft(speed, 45);
    delay(delayTime);
    break;

  case 1010:
    Serial.println("Detected too close to left wall. Turn right 45 degrees!");
    turnRight(speed, 45);
    break;

  case 1011:
    Serial.println("Detected wall in front and left wall. Go back a bit then turn right!");
    goBackward(speed);
    delay(150);
    stop();
    delay(1000);
    turnRight(speed);
    delay(delayTime);
    break;

  case 1100:
    Serial.println("Detected both side wall. Go forward!");
    goForward(speed);
    delay(delayForward);
    break;

  case 1101:
    Serial.println("Detected too close to right wall. Turn left 45 degrees!");
    turnLeft(speed, 45);
    break;

  case 1110:
    Serial.println("Detected too close to left wall. Turn right 45 degrees!");
    turnRight(speed, 45);
    break;

  case 1111:
    Serial.println("Detected dead end. Turn around!");
    turnAround(speed);
    delay(2 * delayTurn);
    break;

  default:
    Serial.println("Default. Go forward");
    goForward(speed);
    delay(delayForward);
    break;
  }
}

void leftEnISR()
{
  leftEnCount++;
}

void rightEnISR()
{
  rightEnCount++;
}