#include <QTRSensors.h>

#define MotFwdL 9   // Motor Forward pin left
#define MotRevL 10  // Motor Reverse pin left

#define MotFwdR 11  // Motor Forward pin right
#define MotRevR 12  // Motor Reverse pin right

int startBtnPin = 2;
int bumpPin = A0;

int enL = 8;
int enR = 13;

int P;
int I;
int D;

const int FORWARD_SPEED = 200;
const int TURN_SPEED = 100;

float Kp = 0.25;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;
int sensorSum = 0;
bool follow_line = true;
const int K = 20;  // adjust K for smooth response

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

float encoderLMS = 0;
float encoderRMS = 0;

bool isEnd = false;
bool readSign = false;
bool followedSign = false;
int countFullLine = 0;
long startTime = 0;
String sign = "";

void setup() {
  Serial.println("Test");
  pinMode(MotFwdL, OUTPUT);
  pinMode(MotRevL, OUTPUT);
  pinMode(MotFwdR, OUTPUT);
  pinMode(MotRevR, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(startBtnPin, INPUT_PULLUP);
  pinMode(bumpPin, INPUT_PULLUP);

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

  Serial.println("Done calibrate!");
  while (digitalRead(startBtnPin) == HIGH) {}
  delay(2000);

  // Stop when push the button
  attachInterrupt(digitalPinToInterrupt(startBtnPin), manualRestart, FALLING);
}

void manualRestart() {
  stop();
  while (digitalRead(startBtnPin) == LOW) {}
  countFullLine = 0;
  readSign = false;
  followedSign = false;
}

void loop() {
  // Hit obstacle
  if (digitalRead(bumpPin) == HIGH) {
    // Move back
    stop();
    delay(1000);
    move('b');
    delay(1000);
  }

  // if ((((sensorSum >= 4000) && (millis() - startTime > 3000)) || (digitalRead(startBtnPin) == LOW)) && (!isEnd)) {  // stop when meet horizontal line
  //   stop();
  //   isEnd = true;
  //   countFullLine++;
  //   delay(500);
  // }

  // Count line
  updateSensorSum();
  if ((sensorSum >= 4000) && (millis() - startTime > 3000)) {
    countFullLine++;
    stop();
    delay(500);
  }

  // Activate read sign when go through line the first time
  if ((countFullLine == 1) && (!readSign)) {
    // Send 'c' to Raspberry Pi
    Serial.println("c");

    //Wait for Raspberry Pi to send sign data
    while ((sign[0] != 'l') && (sign[0] != 'r') && (sign[0] != 's') && (sign[0] != 'a')) {
      if (Serial.available() > 0) {
        sign = Serial.readStringUntil('\n');
        Serial.println(sign);
      }
    }

    startTime = millis();
    readSign = true;
  }

  // Follow sign when go through line the second time
  if ((countFullLine == 2) && (!followedSign)) {
    if (sign[0] == 's') {
      // Stop
      isEnd = true;
    } else if (sign[0] == 'l') {
      // Left
      move('l');
      delay(1000);
      isEnd = false;
    } else if (sign[0] == 'r') {
      // Right
      move('r');
      delay(1000);
      isEnd = false;
    } else if (sign[0] == 'a') {
      // Turn around
      move('l');
      delay(1000);
      move('l');
      delay(1000);
      isEnd = false;
    }

    sign = "";
    startTime = millis();
    followedSign = true;
    stop();
    delay(1000);
  }

  if (!isEnd) {
    if (follow_line) {
      PID_control();
    }

    if (Serial.available() > 0) {
      String lidarData = Serial.readStringUntil('\n');
      Serial.println(lidarData);

      // Found obstacle
      if (lidarData[0] == 'y') {
        // Turn right until found openning infront and the obstacle on the left
        do {
          move('r');
          if (Serial.available() > 0) {
            lidarData = Serial.readStringUntil('\n');
            Serial.println(lidarData);
          }
        } while ((lidarData[0] == 'y') && (lidarData[1] != 'l') && !isOffLine());

        stop();
        delay(2000);

        // Keep distance with the obstacle until back to line
        do {
          lidarData = Serial.readStringUntil('\n');
          if (lidarData[1] == 'c') {
            // Too close to obstacle -> Turn right
            move('r');
            Serial.println("r");
          } else if (lidarData[1] == 'f') {
            // Too far to obstacle -> Turn left
            move('l');
            Serial.println("l");
          } else if (lidarData[1] == 'l') {
            // Perfect distance to obstacle -> Go forward
            move('f');
            Serial.println("f");
          }
        } while (isOffLine());
        stop();
        delay(2000);
      }
    }
  }
}

void stop() {
  digitalWrite(MotFwdL, LOW);
  digitalWrite(MotRevL, LOW);
  digitalWrite(MotFwdR, LOW);
  digitalWrite(MotRevR, LOW);
}

void updateSensorSum() {
  sensorSum = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorSum += sensorValues[i];
  }
}

bool isOffLine() {
  updateSensorSum();
  return sensorSum < 42;
}

void findLeft() {
  while (!isOffLine()) {
    move('l');
  }
  stop();
  delay(1000);

  while (isOffLine()) {
    move('l');
  }
}

void findRight() {
  while (!isOffLine()) {
    move('r');
  }
  stop();
  delay(1000);

  while (isOffLine()) {
    move('r');
  }
}

int PID_control() {
  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   Serial.print(sensorValues[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();
  // updateSensorSum();

  uint16_t positionLine = qtr.readLineBlack(sensorValues);

  int error = 2000 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error;

  int motorSpeedChange = P * Kp + I * Ki + D * Kd;

  int motorSpeedA = FORWARD_SPEED - motorSpeedChange;
  int motorSpeedB = FORWARD_SPEED + motorSpeedChange;

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

void move(char command) {
  if (command == 'l') {
    forward_movement(-TURN_SPEED, TURN_SPEED);
  } else if (command = 'r') {
    forward_movement(TURN_SPEED, -TURN_SPEED);
  } else if (command = 'f') {
    forward_movement(FORWARD_SPEED, FORWARD_SPEED);
  } else if (command = 'b') {
    forward_movement(-FORWARD_SPEED, -FORWARD_SPEED);
  }
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

  analogWrite(enL, speedA);
  analogWrite(enR, speedB);
}
