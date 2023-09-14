//Mega2560
// external interrupt int.0    int.1    int.2   int.3   int.4   int.5            
// pin                  2         3      21      20      19      18

volatile int leftEnCount = 0;
volatile int rightEnCount = 0;

// Left Motor
int enL = 3;
int inL1 = 4;
int inL2 = 5;

// Right motor
int enR = 9;
int inR1 = 6;
int inR2 = 7;

//IF sensor
// int pinLED = 13;
int pinIR_L = 10; //left
int pinIR_R = 9; //right
int pinIR_FL = 12; //front left
int pinIR_FR = 13; //front right 

const int K = 30;  //adjust K for smooth response

void setup()
{
  Serial.begin(9600);

  //configure the IR pin as an input
  pinMode(pinIR_L, INPUT);
  pinMode(pinIR_R, INPUT);
  pinMode(pinIR_FL, INPUT);
  pinMode(pinIR_FR, INPUT);

  // // LED
  // pinMode(pinLED, OUTPUT);

  // interrupt # 5, pin 18
  attachInterrupt(5, leftEnISR, CHANGE); // Also LOW, RISING, FALLING

  // interrupt # 4, pin 19
  attachInterrupt(4, rightEnISR, CHANGE); // Also LOW, RISING, FALLING

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

void loop() {
  // Serial.println(leftEnCount);
  // Serial.println(rightEnCount);
  int sensorL = digitalRead(pinIR_L);
  int sensorR = digitalRead(pinIR_R);
  int sensorFL = digitalRead(pinIR_FL);
  int sensorFR = digitalRead(pinIR_FR);
  
  // 2 front sensors off, 2 side sensors on
  // if (sensorFL == HIGH & sensorFR == HIGH & sensorL == LOW & sensorR == LOW) {
  //   goForward(100);
  //   delay(500);
  // }

  // //turn around if 2 front sensors on, 2 side sensors on (hit wall) (ngõ cụt)
  // if (sensorFL == LOW & sensorFR == LOW & sensorL == LOW & sensorR == LOW) {
  //   Serial.println("turning around");
  //   turnAround();
  //   delay(900);
  //   stop();
  // }

  //move to the left when too close to right wall (both right sensors on)
  if (sensorR == LOW & sensorFR == LOW & sensorL == LOW & sensorFL == HIGH) {
    Serial.println("turning left");
    turnLeft(100);
    delay(100);
  }

  // //move to the right when too close to right wall (both left sensors on)
  if (sensorL == LOW & sensorFL == LOW & sensorR == LOW & sensorFR == HIGH) {
    Serial.println("turning right");
    turnRight(100);
    delay(100);

  }

  //turn right if sensorR low
  // if (sensorR == HIGH) {
  //   turnRight(50);
  // }

  // //turn left if 2 left sensors off, 2 right sensor on
  // if (sensorL == HIGH & sensorFL == LOW & sensorR == LOW & sensorFR == LOW) {
  //   turnLeft(100);
  //   delay(500);
  // }
}

void goForward(int speed) {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

	// For PWM maximum possible values are 0 to 255
	analogWrite(enR, speed);

  int motor_L_speed = speed + K*(rightEnCount-leftEnCount);  
  analogWrite(enL, motor_L_speed);

	// Turn on motor A & B
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, HIGH);
	digitalWrite(inR1, HIGH);
	digitalWrite(inR2, LOW);
}

void stop() {
	// Turn off motors 
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);
}


void turnAround() {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  int speed = 100;
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


void turnRight(int speed) {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  // int speed = 50;
  const int turnWeight = 2;
	analogWrite(enR, speed);

  int motor_L_speed = turnWeight*speed + K*(turnWeight*rightEnCount-leftEnCount);  
  analogWrite(enL, motor_L_speed);

	// Turn on motor A & B
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, HIGH);
	// digitalWrite(inR1, HIGH);
	// digitalWrite(inR2, LOW);  
}

void turnLeft(int speed) {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  // int speed = 50;
  const int turnWeight = 2;
	analogWrite(enL, speed);

  int motor_R_speed = turnWeight*speed + K*(turnWeight*leftEnCount-rightEnCount);  
  analogWrite(enR, motor_R_speed);

	// Turn on motor A & B
	// digitalWrite(inL1, LOW);
	// digitalWrite(inL2, HIGH);
	digitalWrite(inR1, HIGH);
	digitalWrite(inR2, LOW);  
}

void leftEnISR() {
  leftEnCount++;
}

void rightEnISR() {
  rightEnCount++;
}