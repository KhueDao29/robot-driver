#include <QTRSensors.h>

#define MotFwdL 9   // Motor Forward pin left
#define MotRevL 10  // Motor Reverse pin left

#define MotFwdR 11  // Motor Forward pin right
#define MotRevR 12  // Motor Reverse pin right


int encoderPin1 = 21;//10; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 20;//11; //Encoder Otput 'B' must connected with intreput pin of arduino.

int encoderPin3 = 18; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 19; //Encoder Otput 'B' must connected with intreput pin of arduino.

volatile int lastEncodedL = 0; // Here updated value of encoder store.
volatile long encoderValueL = 0; // Raw encoder value

volatile int lastEncodedR = 0; // Here updated value of encoder store.
volatile long encoderValueR = 0; // Raw encoder value

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
bool STOP = false;
const int K = 10;  //adjust K for smooth response

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

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(3, updateEncoderLeft, CHANGE); 
  attachInterrupt(2, updateEncoderLeft, CHANGE);
  attachInterrupt(4, updateEncoderRight, CHANGE); 
  attachInterrupt(5, updateEncoderRight, CHANGE);

  // Serial.println("Calibrating...");
  // for (uint16_t i = 0; i < 400; i++) {
  //   qtr.calibrate();
  // }

  // // print the calibration minimum values measured when emitters were on
  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   Serial.print(qtr.calibrationOn.minimum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  // // print the calibration maximum values measured when emitters were on
  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   Serial.print(qtr.calibrationOn.maximum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  // Serial.println();
  // delay(1000);
}

void loop() {
  // encoderValueL = 0;
  // encoderValueR = 0;
  Serial.print(encoderValueL);
    Serial.print(" ");
  Serial.print(encoderValueR);

  Serial.println();
  // if (STOP == true){
  //   forward_movement(0,0);
  // }
  // else {
  //   PID_control();
  // }
  forward_movement(200, 200+ K*(encoderValueL-lastEncodedR));
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

  const int turnWeight = 1;


  int motorSpeedA = 200 - motorSpeedChange;
  int motorSpeedB = 200 + motorSpeedChange + K*(lastEncodedL-lastEncodedR);
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
    motorSpeedA = 0;
    motorSpeedB = 0;
    STOP = true;
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

void updateEncoderLeft(){
  int MSBL = digitalRead(encoderPin1); //MSB = most significant bit
  int LSBL = digitalRead(encoderPin2); //LSB = least significant bit
  // int MSBR = digitalRead(encoderPin3); //MSB = most significant bit
  // int LSBR = digitalRead(encoderPin4); //LSB = least significant bit

  int encodedL = (MSBL << 1) |LSBL; //converting the 2 pin value to single number
  int sumL  = (lastEncodedL << 2) | encodedL; //adding it to the previous encoded value

  if(sumL == 0b1101 || sumL == 0b0100 || sumL == 0b0010 || sumL == 0b1011) encoderValueL --;
  if(sumL == 0b1110 || sumL == 0b0111 || sumL == 0b0001 || sumL == 0b1000) encoderValueL ++;

  lastEncodedL = encodedL; //store this value for next time

}

void updateEncoderRight(){
//   int MSBL = digitalRead(encoderPin1); //MSB = most significant bit
  // int LSBL = digitalRead(encoderPin2); //LSB = least significant bit
  int MSBR = digitalRead(encoderPin3); //MSB = most significant bit
  int LSBR = digitalRead(encoderPin4); //LSB = least significant bit


  int encodedR = (MSBR << 1) |LSBR; //converting the 2 pin value to single number
  int sumR  = (lastEncodedR << 2) | encodedR; //adding it to the previous encoded value

  if(sumR == 0b1101 || sumR == 0b0100 || sumR == 0b0010 || sumR == 0b1011) encoderValueR --;
  if(sumR == 0b1110 || sumR == 0b0111 || sumR == 0b0001 || sumR == 0b1000) encoderValueR ++;

  lastEncodedR = encodedR; //store this value for next time

}
