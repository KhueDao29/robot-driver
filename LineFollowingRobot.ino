

#define MotFwdL  7  // Motor Forward pin left
#define MotRevL  8 // Motor Reverse pin left 

#define MotFwdR  5  // Motor Forward pin right
#define MotRevR  6 // Motor Reverse pin right

int encoderPin1 = 2;//10; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3;//11; //Encoder Otput 'B' must connected with intreput pin of arduino.

int encoderPin3 = 10; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 11; //Encoder Otput 'B' must connected with intreput pin of arduino.

volatile int lastEncodedL = 0; // Here updated value of encoder store.
volatile long encoderValueL = 0; // Raw encoder value

volatile int lastEncodedR = 0; // Here updated value of encoder store.
volatile long encoderValueR = 0; // Raw encoder value

void setup() {

  pinMode(MotFwdL, OUTPUT); 
  pinMode(MotRevL, OUTPUT); 
  pinMode(MotFwdR, OUTPUT); 
  pinMode(MotRevR, OUTPUT); 
  Serial.begin(9600); //initialize serial comunication

  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP); 
  pinMode(encoderPin4, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin3, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin4, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);


}

void loop() {
    digitalWrite(MotFwdL, LOW); 
    digitalWrite(MotRevL, HIGH);
    digitalWrite(MotFwdR, HIGH); 
    digitalWrite(MotRevR, LOW);
    Serial.print("Forward  ");
    Serial.println(encoderValueR);

  // for (int i = 0; i <= 500; i++){
  //   digitalWrite(MotFwd, LOW); 
  //   digitalWrite(MotRev, HIGH);
  //   Serial.print("Forward  ");
  //   Serial.println(encoderValue);
  // }

  // delay(1000);

  // for (int i = 0; i <= 500; i++){
  //   digitalWrite(MotFwd, HIGH); 
  //   digitalWrite(MotRev, LOW);
  //   Serial.print("Reverse  ");
  //   Serial.println(encoderValue);

} 

void updateEncoder(){
  int MSBL = digitalRead(encoderPin1); //MSB = most significant bit
  int LSBL = digitalRead(encoderPin2); //LSB = least significant bit
  int MSBR = digitalRead(encoderPin3); //MSB = most significant bit
  int LSBR = digitalRead(encoderPin4); //LSB = least significant bit

  int encodedL = (MSBL << 1) |LSBL; //converting the 2 pin value to single number
  int sumL  = (lastEncodedL << 2) | encodedL; //adding it to the previous encoded value

  if(sumL == 0b1101 || sumL == 0b0100 || sumL == 0b0010 || sumL == 0b1011) encoderValueL --;
  if(sumL == 0b1110 || sumL == 0b0111 || sumL == 0b0001 || sumL == 0b1000) encoderValueL ++;

  lastEncodedL = encodedL; //store this value for next time

  int encodedR = (MSBR << 1) |LSBR; //converting the 2 pin value to single number
  int sumR  = (lastEncodedR << 2) | encodedR; //adding it to the previous encoded value

  if(sumR == 0b1101 || sumR == 0b0100 || sumR == 0b0010 || sumR == 0b1011) encoderValueR --;
  if(sumR == 0b1110 || sumR == 0b0111 || sumR == 0b0001 || sumR == 0b1000) encoderValueR ++;

  lastEncodedR = encodedR; //store this value for next time

}