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
int pinIR_F = A0; // front

const int K = 30; // adjust K for smooth response

const int speed = 200;
const int delayTime = 300;
const int delayTurn = 180;
const int delayForward = 200;

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

void turnRight(int speed, int degree = 90, int sensorF = HIGH)
{
  turnAround(-speed);
  delay(int(delayTurn * degree / 90));
  stop();
  delay(1000);
  if(sensorF == HIGH) {
    goForward(speed);;
  }
  // // Reset encoder counter
  // rightEnCount = 0;
  // leftEnCount = 0;

  // // int speed = 50;
  // const int turnWeight = 3;
  // if (speed >= 0)
  // {
  //   analogWrite(enR, speed);

  //   int motor_L_speed = turnWeight * speed + K * (turnWeight * rightEnCount - leftEnCount);
  //   analogWrite(enL, motor_L_speed);

  //   // Turn on motor A & B
  //   digitalWrite(inL1, LOW);
  //   digitalWrite(inL2, HIGH);
  //   digitalWrite(inR1, HIGH);
  //   digitalWrite(inR2, LOW);
  // }
  // else
  // {
  //   analogWrite(enR, -speed);

  //   int motor_L_speed = turnWeight * -speed + K * (turnWeight * rightEnCount - leftEnCount);
  //   analogWrite(enL, motor_L_speed);

  //   // Turn on motor A & B
  //   digitalWrite(inL1, HIGH);
  //   digitalWrite(inL2, LOW);
  //   digitalWrite(inR1, LOW);
  //   digitalWrite(inR2, HIGH);
  // }
}

void loop()
{
  // Serial.println(leftEnCount);
  // Serial.println(rightEnCount);
  int sensorL = digitalRead(pinIR_L);
  int sensorR = digitalRead(pinIR_R);
  int sensorFL = digitalRead(pinIR_FL);
  int sensorFR = digitalRead(pinIR_FR);
  int sensorF = digitalRead(pinIR_F);

  // Serial.println("sensor L = " + sensorL);
  // Serial.println("sensor R = " + sensorR);
  // Serial.println("sensor FL = " + sensorFL);
  // Serial.println("sensor FR = " + sensorFR);

  String sensorValue = readSensor(sensorL, sensorR, sensorFL, sensorFR);
  Serial.print(sensorValue + " ");
  
  if(sensorF == LOW & sensorValue == 0) {
    goBackward(speed);
    delay(150);
    stop();
    delay(1000);
    turnRight(speed);
    delay(delayTime);
  } else {
    action(sensorValue, sensorF);
  }
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

void goBackward(int speed)
{
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  // For PWM maximum possible values are 0 to 255
  analogWrite(enR, speed);

  int motor_L_speed = speed + K * (rightEnCount - leftEnCount);
  analogWrite(enL, motor_L_speed);

  // Turn on motor A & B
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
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

  if (speed >= 0)
  {
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
  else
  {
    analogWrite(enR, -speed);

    // int motor_L_speed = turnWeight*speed + K*(turnWeight*rightEnCount-leftEnCount);
    // analogWrite(enL, motor_L_speed);
    analogWrite(enL, -speed);

    // Turn on motor A & B
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
  }
}



void turnLeft(int speed, int degree = 90, int sensorF = HIGH)
{
  turnAround(speed);
  delay(int(delayTurn * degree / 90));
  stop();
  delay(1000);
  if(sensorF == HIGH) {
    goForward(speed);;
  }
  // // Reset encoder counter
  // rightEnCount = 0;
  // leftEnCount = 0;

  // // int speed = 50;
  // const int turnWeight = 3;
  // if (speed >= 0)
  // {

  //   analogWrite(enL, speed);

  //   int motor_R_speed = turnWeight * speed + K * (turnWeight * leftEnCount - rightEnCount);
  //   analogWrite(enR, motor_R_speed);

  //   // Turn on motor A & B
  //   digitalWrite(inL1, LOW);
  //   digitalWrite(inL2, HIGH);
  //   digitalWrite(inR1, HIGH);
  //   digitalWrite(inR2, LOW);
  // }
  // else
  // {
  //   analogWrite(enL, -speed);

  //   int motor_R_speed = turnWeight * -speed + K * (turnWeight * leftEnCount - rightEnCount);
  //   analogWrite(enR, motor_R_speed);

  //   // Turn on motor A & B
  //   digitalWrite(inL1, HIGH);
  //   digitalWrite(inL2, LOW);
  //   digitalWrite(inR1, LOW);
  //   digitalWrite(inR2, HIGH);
  // }
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



void action(String sensorValue, int sensorF)
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
    turnRight(speed, 45, sensorF);
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
