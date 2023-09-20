// Mega2560
//  external interrupt int.0    int.1    int.2   int.3   int.4   int.5
//  pin                  2         3      21      20      19      18

volatile int leftEnCount = 0;
volatile int rightEnCount = 0;

// Left Motor
int enL = 4;
int inL1 = 5;
int inL2 = 6;

// Right motor
int inR1 = 7;
int inR2 = 8;
int enR = 9;

// IF sensor
int pinIR_L = 10;  // left
int pinIR_FL = 11; // front left
int pinIR_R = 12;  // right
int pinIR_FR = 13; // front right

const int K = 30; // adjust K for smooth response

void setup()
{
  Serial.begin(9600);

  // configure the IR pin as an input
  pinMode(pinIR_L, INPUT);
  pinMode(pinIR_R, INPUT);
  pinMode(pinIR_FL, INPUT);
  pinMode(pinIR_FR, INPUT);

  // interrupt # 5, pin 18
  attachInterrupt(1, leftEnISR, CHANGE); // Also LOW, RISING, FALLING

  // interrupt # 4, pin 19
  attachInterrupt(0, rightEnISR, CHANGE); // Also LOW, RISING, FALLING

  // Set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void loop()
{
  // Serial.println(leftEnCount);
  // Serial.println(rightEnCount);
  int sensorL = digitalRead(pinIR_L);
  int sensorR = digitalRead(pinIR_R);
  int sensorFL = digitalRead(pinIR_FL);
  int sensorFR = digitalRead(pinIR_FR);

  // Serial.println("sensor L = " + sensorL);
  // Serial.println("sensor R = " + sensorR);
  // Serial.println("sensor FL = " + sensorFL);
  // Serial.println("sensor FR = " + sensorFR);

  String sensorValue = readSensor(sensorL, sensorR, sensorFL, sensorFR);
  Serial.print(sensorValue + " ");
  action(sensorValue);
  stop();
  delay(1000);
}

void goForward(int speed)
{
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  // For PWM maximum possible values are 0 to 255
  analogWrite(enR, speed);

  int motor_L_speed = speed + K * (rightEnCount - leftEnCount);
  analogWrite(enL, motor_L_speed);

  // Turn on motor A & B
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void stop()
{
  // Turn off motors
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void turnAround(int speed)
{
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  // const int turnWeight = 2;
  analogWrite(enR, speed);

  // int motor_L_speed = turnWeight*speed + K*(turnWeight*rightEnCount-leftEnCount);
  // analogWrite(enL, motor_L_speed);
  analogWrite(enL, speed);

  // Turn on motor A & B
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}

void turnRight(int speed)
{
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  // int speed = 50;
  const int turnWeight = 1;
  analogWrite(enR, speed);

  int motor_L_speed = turnWeight * speed + K * (turnWeight * rightEnCount - leftEnCount);
  analogWrite(enL, motor_L_speed);

  // Turn on motor A & B
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void turnLeft(int speed)
{
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  // int speed = 50;
  const int turnWeight = 1;
  analogWrite(enL, speed);

  int motor_R_speed = turnWeight * speed + K * (turnWeight * leftEnCount - rightEnCount);
  analogWrite(enR, motor_R_speed);

  // Turn on motor A & B
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

String readSensor(int sensorL, int sensorR, int sensorFL, int sensorFR)
{
  if (sensorL == HIGH)
  {
    sensorL = 0;
  }
  else
  {
    sensorL = 1;
  }

  if (sensorR == HIGH)
  {
    sensorR = 0;
  }
  else
  {
    sensorR = 1;
  }

  if (sensorFL == HIGH)
  {
    sensorFL = 0;
  }
  else
  {
    sensorFL = 1;
  }

  if (sensorFR == HIGH)
  {
    sensorFR = 0;
  }
  else
  {
    sensorFR = 1;
  }

  return String(sensorL) + String(sensorR) + String(sensorFL) + String(sensorFR);
}

void action(String sensorValue)
{
  const int speed = 200;
  const int delayTime = 500;
  const int value = sensorValue.toInt();
  switch (value)
  {
  case 0: //0000
    goForward(speed);
    delay(delayTime);
    Serial.println("Go forward!");
    break;

  case 1: //0001
    Serial.println("Detected wall in front at a right angle. Turn left!");
    // goForward(speed);
    turnLeft(speed);
    delay(delayTime);
    break;
  case 10: //0010
    Serial.println("Detected wall in front at a left angle. Turn left!");
    turnLeft(speed);
    delay(delayTime);
    break;
  case 11: //0011
    Serial.println("Dectected wall in front with both side wall open. Turn left!");
    turnLeft(speed);
    delay(delayTime);
    break;
  case 100: //0100
    Serial.println("Dectected left wall open. Turn left!");
    turnLeft(speed);
    delay(delayTime);
    break;
  case 101: //0101
    Serial.println("Dectected too close to right wall. Turn left a bit!");
    turnLeft(120);
    delay(300);
    break;
  case 110: //0110
    Serial.println("Detected wall in front at a right angle but too close to right wall. Turn right a bit then left!");
    turnRight(speed);
    delay(200);
    turnLeft(speed);
    delay(delayTime);
    break;
  case 111: //0111
    Serial.println("Detected wall in front and left wall open. Turn left!");
    turnLeft(speed);
    delay(delayTime);
    break;
  case 1000:
    Serial.println("Detected left wall close and in front open. Go forward!");
    goForward(speed);
    delay(delayTime);
    break;
  case 1001:
    Serial.println("Detected wall in front at a left angle but too close to left wall. Turn left a bit then right!");
    turnLeft(speed);
    delay(200);
    turnRight(speed);
    delay(delayTime);
    break;
  case 1010:
    Serial.println("Detected too close to left wall. Turn right a bit!");
    turnRight(speed);
    delay(100);
    break;
  case 1011:
    Serial.println("Detected wall in front and right wall open. Turn right!");
    // turnAround();
    turnRight(speed);
    delay(delayTime);
    break;
  case 1100:
    Serial.println("Detected both side wall. Go forward!");
    goForward(speed);
    delay(delayTime);
    break;
  case 1101:
    Serial.println("Detected too close to right wall. Turn left a bit!");
    turnLeft(speed);
    delay(100);
    break;
  case 1110:
    Serial.println("Detected too close to left wall. Turn right a bit!");
    turnRight(speed);
    delay(100);
    break;
  case 1111:
    Serial.println("Detected dead end. Turn around!");
    turnAround(speed);
    delay(delayTime);
    break;
  default:
    Serial.println("Default. Go forward");
    goForward(speed);
    delay(delayTime);
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