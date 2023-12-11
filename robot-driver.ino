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

float Kp = 0.3;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;
int sensorSum = 0;
bool nextPhase = false;
bool STOP = false;
bool follow_line = true;
const int K = 20;  // adjust K for smooth response

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

float encoderLMS = 0;
float encoderRMS = 0;

bool isEnd = false;
bool disablePID = false;
bool readSign = false;

long startTime = 0;

void setup() {
  Serial.println("Test");
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

  if (!disablePID) {
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
  }
  while (digitalRead(startBtnPin) == HIGH) {}
  delay(2000);
}

void loop() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  // Serial.print("Sensors: ");

  sensorSum = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    // Serial.print(sensorValues[i]);
    // Serial.print('\t');
    sensorSum += sensorValues[i];
  }

  if (isEnd && (digitalRead(startBtnPin) == LOW)) {
    isEnd = false;
    delay(2000);
  }

  if ((((sensorSum >= 4000) && !disablePID && (millis() - startTime > 5000)) || (digitalRead(startBtnPin) == LOW)) && (!isEnd)) { //stop when meet horizontal line
    stop();
    isEnd = true;
    // delay(500);
    startTime = 0;
    readSign = !readSign;
    if (readSign) {
      //after reaching horizontal line -> stop to read traffic sign 
      if (Serial.available() > 0) {
          String sign = Serial.readStringUntil('\n');
          Serial.println('c');
          while (true) {

            sign = Serial.readStringUntil('\n');
            if (sign != 's' ||sign != 'r' ||sign != 'l' ) {
              continue;
            }
            Serial.println(sign); 
            if (sign == 's') {
              //PID
              // readSign = false;
              startTime = millis();
              break;
            } else if (sign == 'l') {
              //move forward, reach horizontal line, then turn left 
              startTime = millis();
              while (sensorSum < 4000) {
                forward_movement(200, 200);
              }
              stop();
              forward_movement(100, -100);
              break;
              // readSign = false;
            } else if (sign == 'r') {
              //move forward, reach horizontal line, then turn right 
              startTime = millis();
              while (sensorSum < 4000) {
                forward_movement(200, 200);
              }
              stop();
              forward_movement(-100, 100);
              break;
              // readSign = false;
            }
          }
          
      } 
    
    } 
  }



  if (!isEnd) {
    

    if (!disablePID) {
      if (follow_line) {
        PID_control();
      }
    } else {
      forward_movement(200, 200);
    }

    if (Serial.available() > 0) {
      String lidarData = Serial.readStringUntil('\n');
      Serial.println(lidarData);

      if (lidarData[0] == 'y') {

        while ((lidarData[0] == 'y') && (lidarData[1] != 'l')) {
          if (digitalRead(startBtnPin) == LOW) {
            stop();
            isEnd = true;
          }
          forward_movement(200, -200);
          lidarData = Serial.readStringUntil('\n');
          Serial.println(lidarData);
        }

        stop();
        delay(2000);

        do {
          uint16_t positionLine = qtr.readLineBlack(sensorValues);

          sensorSum = 0;
          for (uint8_t i = 0; i < SensorCount; i++) {
            sensorSum += sensorValues[i];
          }

          if (digitalRead(startBtnPin) == LOW) {
            stop();
            isEnd = true;
            break;
          }
          if (lidarData[1] == 'c') {
            forward_movement(75, -75);
            Serial.println("r");
          } else if (lidarData[1] == 'f') {
            forward_movement(-75, 75);
            Serial.println("l");
          } else if (lidarData[1] == 'l') {
            forward_movement(200, 200);
            Serial.println("s");
          }
          lidarData = Serial.readStringUntil('\n');
        } while (isOffLine());
        stop();
        delay(2000);
        startTime = millis();
      }
    }
  }


    if (digitalRead(startBtnPin) == LOW) {
      stop();
      isEnd = true;
    }

}



void stop() {
  digitalWrite(MotFwdL, LOW);
  digitalWrite(MotRevL, LOW);
  digitalWrite(MotFwdR, LOW);
  digitalWrite(MotRevR, LOW);
}

bool isOffLine() {
  return sensorSum < 500;
}

int PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  // Serial.print("Sensors: ");

  sensorSum = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    // Serial.print(sensorValues[i]);
    // Serial.print('\t');
    sensorSum += sensorValues[i];
  }
  // Serial.print("Position: ");
  // Serial.print(positionLine);
  // Serial.print('\t');

  int error = 2000 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error;

  int motorSpeedChange = P * Kp + I * Ki + D * Kd;
  // Serial.print("Error: ");
  // Serial.print(motorSpeedChange);
  // Serial.print("\t");

  int motorSpeedA = 200 - motorSpeedChange;
  int motorSpeedB = 200 + motorSpeedChange;
  // Serial.print("Speed Left:");
  // Serial.print(motorSpeedA);
  // Serial.print(" ");
  // Serial.print("Speed Right:");
  // Serial.println(motorSpeedB);

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
  // speedB += K * (encoderValueL - encoderValueR);
  analogWrite(enR, speedB);
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
