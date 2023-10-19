#include <QTRSensors.h>

#define MotFwdL 9   // Motor Forward pin left
#define MotRevL 10  // Motor Reverse pin left

#define MotFwdR 11  // Motor Forward pin right
#define MotRevR 12  // Motor Reverse pin right

int encoderPin1 = 21;  // 10; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 20;  // 11; //Encoder Otput 'B' must connected with intreput pin of arduino.

int encoderPin3 = 18;  // Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 19;  // Encoder Otput 'B' must connected with intreput pin of arduino.

int startBtnPin = 7;

volatile int lastEncodedL = 0;    // Here updated value of encoder store.
volatile long encoderValueL = 0;  // Raw encoder value

volatile int lastEncodedR = 0;    // Here updated value of encoder store.
volatile long encoderValueR = 0;  // Raw encoder value

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
int sensorSum = 0;
bool nextPhase = false;
bool STOP = false;
const int K = 20;  // adjust K for smooth response

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

const double target[3] = { 0.77, 2, 90 * PI / 180 };
double currentLocation[3] = { 0, 0, 0 };
double lastDotLocation[3] = { 0, 0, 0 };
const int gamma = 3;
const int lambda = 6;
const int h = 1;
const float wheelDistance = 0.16;
double vl = 0;
double vr = 0;

float encoderLMS = 0;
float encoderRMS = 0;

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
  pinMode(startBtnPin, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH);  // turn pullup resistor on
  digitalWrite(encoderPin2, HIGH);  // turn pullup resistor on
  digitalWrite(encoderPin3, HIGH);  // turn pullup resistor on
  digitalWrite(encoderPin4, HIGH);  // turn pullup resistor on
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A2, A3, A4, A5, A6 }, SensorCount);

  // call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(2, updateEncoder, CHANGE);
  attachInterrupt(3, updateEncoder, CHANGE);
  attachInterrupt(4, updateEncoder, CHANGE);
  attachInterrupt(5, updateEncoder, CHANGE);

  while (digitalRead(startBtnPin) == HIGH) {}
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
  while (digitalRead(startBtnPin) == HIGH) {}

  const float WHEEL_RADIUS = 0.045 / 2;
  const int GEAR_RATIO = 150;
  const int PPR = 7 * GEAR_RATIO;
  encoderValueL = 0;
  encoderValueR = 0;
  const float PPS2MS = 2 * PI * WHEEL_RADIUS / PPR * GEAR_RATIO;
  forward_movement(255, 255);
  delay(1000);
  encoderLMS = encoderValueL * PPS2MS;
  encoderRMS = encoderValueR * PPS2MS;
  Serial.print(encoderLMS);
  Serial.print(" ");
  Serial.println(encoderRMS);
  stop();

  while (digitalRead(startBtnPin) == HIGH) {}
  Serial.println();
  delay(1000);
}

void loop() {
  long currentTime = millis();
  encoderValueL = 0;
  encoderValueR = 0;

  if (nextPhase == true) {
    if (array_cmp(currentLocation, target, 3, 3) == true) {
      stop();
    } else {
      updateController();
      Serial.print("VL: ");
      Serial.print(vl);
      Serial.print(" ");
      Serial.print("VR: ");
      Serial.print(vr);
      Serial.print("\t");

      int motorSpeedLeft = int(255 * vl / encoderLMS) * 2 + 200;
      int motorSpeedRight = int(255 * vr / encoderRMS) * 2 + 200;
      Serial.print(motorSpeedLeft);
      Serial.print(" ");
      Serial.print(motorSpeedRight);
      Serial.print("\t");

      forward_movement(motorSpeedLeft, motorSpeedRight);
      updateLocation(currentTime);
      Serial.print("X: ");
      Serial.print(currentLocation[0]);
      Serial.print(" Y: ");
      Serial.print(currentLocation[1]);
      Serial.print(" Theta: ");
      Serial.println(currentLocation[2]);
    }
  } else {
    PID_control();
  }
}

boolean array_cmp(double *a, double *b, int len_a, int len_b) {
  int n;

  // if their lengths are different, return false
  if (len_a != len_b) return false;

  // test each element to be the same. if not, return false
  for (n = 0; n < len_a; n++)
    if (abs(a[n] - b[n]) >= 0.01) return false;

  //ok, if we have not returned yet, they are equal :)
  return true;
}

void stop() {
  digitalWrite(MotFwdL, LOW);
  digitalWrite(MotRevL, LOW);
  digitalWrite(MotFwdR, LOW);
  digitalWrite(MotRevR, LOW);
}

int PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  Serial.print("Sensors: ");

  sensorSum = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
    sensorSum += sensorValues[i];
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
  if (sensorSum >= 4000) {
    Serial.println("Change to auto drive phase!");
    motorSpeedA = 0;
    motorSpeedB = 0;
    nextPhase = true;
  }

  forward_movement(motorSpeedA, motorSpeedB);
}

void forward_movement(int speedA, int speedB) {
  if (speedA >= 0) {
    digitalWrite(MotFwdL, HIGH);
    digitalWrite(MotRevL, LOW);
  } else {
    digitalWrite(MotFwdL, LOW);
    digitalWrite(MotRevL, HIGH);
    speedA = -speedA;
  }

  if (speedB >= 0) {
    digitalWrite(MotFwdR, HIGH);
    digitalWrite(MotRevR, LOW);
  } else {
    digitalWrite(MotFwdR, LOW);
    digitalWrite(MotRevR, HIGH);
    speedB = -speedB;
  }

  if (speedA > 255) {
    speedA = 255;
  }
  if (speedB > 255) {
    speedB = 255;
  }

  if (speedA < 100) {
    speedA = 100;
  }
  if (speedB > 100) {
    speedB = 100;
  }

  analogWrite(enL, speedA);
  speedB += K * (encoderValueL - encoderValueR);
  analogWrite(enR, speedB);
}

void updateController() {
  double rho = sqrt(pow(target[0] - currentLocation[0], 2) + pow(target[1] - currentLocation[1], 2));
  double phi = atan2(target[1] - currentLocation[1], target[0] - currentLocation[0]) - target[2];
  double alpha = phi + target[2] - currentLocation[2];

  double v = gamma * cos(alpha) * rho;
  double w = lambda * alpha + gamma * cos(alpha) * sin(alpha) / alpha * (alpha + h * phi);

  vl = (2 * v - wheelDistance * w) / 2;
  vr = (2 * v + wheelDistance * w) / 2;
}

void updateLocation(long pastTime) {
  double xdot = (vr + vl) / 2 * cos(currentLocation[2]);
  double ydot = (vr + vl) / 2 * sin(currentLocation[2]);
  double thetadot = (vr - vl) / wheelDistance;

  float deltaT = float(millis() - pastTime) / 1000.0;
  Serial.print("Time: ");
  Serial.print(deltaT, 10);
  Serial.print("\t");
  currentLocation[0] += xdot * deltaT;
  currentLocation[1] += ydot * deltaT;
  currentLocation[2] += thetadot * deltaT;
}

void updateEncoder() {
  int MSBL = digitalRead(encoderPin1);  // MSB = most significant bit
  int LSBL = digitalRead(encoderPin2);  // LSB = least significant bit
  int MSBR = digitalRead(encoderPin3);  // MSB = most significant bit
  int LSBR = digitalRead(encoderPin4);  // LSB = least significant bit

  int encodedL = (MSBL << 1) | LSBL;          // converting the 2 pin value to single number
  int sumL = (lastEncodedL << 2) | encodedL;  // adding it to the previous encoded value

  if (sumL == 0b1101 || sumL == 0b0100 || sumL == 0b0010 || sumL == 0b1011)
    encoderValueL--;
  if (sumL == 0b1110 || sumL == 0b0111 || sumL == 0b0001 || sumL == 0b1000)
    encoderValueL++;

  lastEncodedL = encodedL;  // store this value for next time

  int encodedR = (MSBR << 1) | LSBR;          // converting the 2 pin value to single number
  int sumR = (lastEncodedR << 2) | encodedR;  // adding it to the previous encoded value

  if (sumR == 0b1101 || sumR == 0b0100 || sumR == 0b0010 || sumR == 0b1011)
    encoderValueR--;
  if (sumR == 0b1110 || sumR == 0b0111 || sumR == 0b0001 || sumR == 0b1000)
    encoderValueR++;

  lastEncodedR = encodedR;  // store this value for next time
}
