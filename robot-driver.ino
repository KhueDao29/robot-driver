

#define MotFwdL 9 // Motor Forward pin left
#define MotRevL 10 // Motor Reverse pin left

#define MotFwdR 11 // Motor Forward pin right
#define MotRevR 12 // Motor Reverse pin right


int encoderPin1 = 20; // Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 21; // Encoder Otput 'B' must connected with intreput pin of arduino.
int encoderPin3 = 18; // 10; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 19; // 11; //Encoder Otput 'B' must connected with intreput pin of arduino.

volatile int lastEncodedL = 0;   // Here updated value of encoder store.
volatile long encoderValueL = 0; // Raw encoder value

volatile int lastEncodedR = 0;   // Here updated value of encoder store.
volatile long encoderValueR = 0; // Raw encoder value

const int K = 20; // adjust K for smooth response

void setup()
{

  pinMode(MotFwdL, OUTPUT);
  pinMode(MotRevL, OUTPUT);
  pinMode(MotFwdR, OUTPUT);
  pinMode(MotRevR, OUTPUT);
  Serial.begin(9600); // initialize serial comunication

  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP);
  pinMode(encoderPin4, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); // turn pullup resistor on
  digitalWrite(encoderPin3, HIGH); // turn pullup resistor on
  digitalWrite(encoderPin4, HIGH); // turn pullup resistor on

  // call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(2, updateEncoderL, CHANGE);
  attachInterrupt(3, updateEncoderL, CHANGE);
  attachInterrupt(4, updateEncoderR, CHANGE);
  attachInterrupt(5, updateEncoderR, CHANGE);
}

void loop()
{
  digitalWrite(MotFwdL, HIGH);
  digitalWrite(MotRevL, LOW);
  digitalWrite(MotFwdR, HIGH);
  digitalWrite(MotRevR, LOW);
  Serial.print("Forward  ");
  Serial.print("Left: ");
  Serial.print(encoderValueL);
  Serial.print("  ");
  Serial.print("Right: ");
  Serial.println(encoderValueR);

  // for (int i = 0; i <= 500; i++){
  //   digitalWrite(MotFwd, LOW);
  //   digitalWrite(MotRev, HIGH);
  //   digitalWrite(MotFwdR, LOW);
  //   digitalWrite(MotRevR, HIGH);
  //   Serial.print("Forward  ");
  //   Serial.println(encoderValue);
  // }

  // delay(1000);

  // for (int i = 0; i <= 500; i++){
  //   digitalWrite(MotFwdL, HIGH);
  //   digitalWrite(MotRevL, LOW);
  //   digitalWrite(MotFwdR, HIGH);
  //   digitalWrite(MotRevR, LOW);
  //   Serial.print("Reverse  ");
  //   Serial.println(encoderValue);
  // }
}

// function to make robot move forward
// Parameter:
//   speed: speed of the robot

void moveForward(int speed)
{
  // Reset encoder counter
  lastEncodedL = 0;  // Here updated value of encoder store.
  encoderValueL = 0; // Raw encoder value

  lastEncodedR = 0;  // Here updated value of encoder store.
  encoderValueR = 0; // Raw encoder value

  // For PWM maximum possible values are 0 to 255
  analogWrite(encoderValueR, speed);

  int motor_L_speed = speed + K * (encoderValueR - encoderValueL);
  analogWrite(encoderValueL, motor_L_speed);

  // Turn on motor A & B
  digitalWrite(MotFwdL, LOW);
  digitalWrite(MotRevL, HIGH);
  digitalWrite(MotFwdR, LOW);
  digitalWrite(MotRevR, HIGH);
  Serial.print("Forward  ");
  Serial.println(encoderValueR);
}

void updateEncoderL()
{
  int MSBL = digitalRead(encoderPin1); // MSB = most significant bit
  int LSBL = digitalRead(encoderPin2); // LSB = least significant bit

  int encodedL = (MSBL << 1) | LSBL;         // converting the 2 pin value to single number
  int sumL = (lastEncodedL << 2) | encodedL; // adding it to the previous encoded value

  if (sumL == 0b1101 || sumL == 0b0100 || sumL == 0b0010 || sumL == 0b1011)
    encoderValueL--;
  if (sumL == 0b1110 || sumL == 0b0111 || sumL == 0b0001 || sumL == 0b1000)
    encoderValueL++;

  lastEncodedL = encodedL; // store this value for next time
}

void updateEncoderR() {
  int MSBR = digitalRead(encoderPin3); // MSB = most significant bit
  int LSBR = digitalRead(encoderPin4); // LSB = least significant bit
  int encodedR = (MSBR << 1) | LSBR;         // converting the 2 pin value to single number
  int sumR = (lastEncodedR << 2) | encodedR; // adding it to the previous encoded value

  if (sumR == 0b1101 || sumR == 0b0100 || sumR == 0b0010 || sumR == 0b1011)
    encoderValueR--;
  if (sumR == 0b1110 || sumR == 0b0111 || sumR == 0b0001 || sumR == 0b1000)
    encoderValueR++;

  lastEncodedR = encodedR; // store this value for next time
}
