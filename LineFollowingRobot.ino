

#define MotFwdL  9  // Motor Forward pin left
#define MotRevL  10 // Motor Reverse pin left 

#define MotFwdR  11  // Motor Forward pin right
#define MotRevR  12 // Motor Reverse pin right

int encoderPin1 = 20;//10; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 21;//11; //Encoder Otput 'B' must connected with intreput pin of arduino.

int encoderPin3 = 18; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 19; //Encoder Otput 'B' must connected with intreput pin of arduino.

int enL = 8;
int enR = 13;

volatile int lastEncodedL = 0; // Here updated value of encoder store.
volatile long encoderValueL = 0; // Raw encoder value

volatile int lastEncodedR = 0; // Here updated value of encoder store.
volatile long encoderValueR = 0; // Raw encoder value

const int K = 30; // adjust K for smooth response
const int SPEED = 150; //default speed
const int TURN_WEIGHT = 3;

#define set_point 2000
#define max_speed 250 //set Max Speed Value
#define Kp 0 //set Kp Value
#define Ki 0 //set Ki Value
#define Kd 0 //set Kd Value

int proportional=0;
int integral=0;
int derivative=0;
int last_proportional=0;
int right_speed=0;
int left_speed=0;
int sensors_sum=0;
int sensors_average=0;
int sensors[5]={0,0,0,0,0};
int Position=0;
int error_value=0;


void setup() {

  pinMode(MotFwdL, OUTPUT); 
  pinMode(MotRevL, OUTPUT); 
  pinMode(MotFwdR, OUTPUT); 
  pinMode(MotRevR, OUTPUT); 
  Serial.begin(9600); //initialize serial comunication

  pinMode(enR, OUTPUT);
	pinMode(enL, OUTPUT);
  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP); 
  pinMode(encoderPin4, INPUT_PULLUP);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin3, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin4, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(2, updateEncoder, CHANGE); 
  attachInterrupt(3, updateEncoder, CHANGE);
  attachInterrupt(4, updateEncoder, CHANGE); 
  attachInterrupt(5, updateEncoder, CHANGE);


}

void loop() {
    // moveForward(SPEED);
int  sensors_average = 0; 
  sensors_sum = 0;
 
  for (int i = 2; i <=6; i++) 
   {
    sensors[i] = digitalRead(i); 
    sensors_average += sensors[i] * (i-2) * 1000   ;  
    sensors_sum += sensors[i];
    Serial.print(sensors[i]);
    Serial.print(" ");
    }
  Serial.println();

  if(sensors_sum==0)
  {
    Stop();    
  }
  if (sensors_sum >= 3500)
  {
   moveForward(SPEED);
  }
               
  if(sensors_sum < 3500 && sensors_sum > 0)
      {
        Position = int(sensors_average / sensors_sum);
        pid_calc();
        calc_turn();
        motor_drive(right_speed*3/5,left_speed*1.7);
      }
  
  
 delay(500); 
}


void Stop()
{
  analogWrite(enL,0);
  analogWrite(enR,0);
  delay(500);
}


// function to make robot move forward
// Parameter:
//    speed: speed of the robot

void moveForward(int speed){
    // Reset encoder counter
    lastEncodedL = 0; // Here updated value of encoder store.
    encoderValueL = 0; // Raw encoder value

    lastEncodedR = 0; // Here updated value of encoder store.
    encoderValueR = 0; // Raw encoder value

    // For PWM maximum possible values are 0 to 255
    analogWrite(enR, speed);

    int motor_L_speed = speed + K * (encoderValueR - encoderValueL);
    analogWrite(enL, motor_L_speed);

    // Serial.println(speed);
    // Serial.println(motor_L_speed);
    Serial.println(encoderValueR);
    Serial.println(encoderValueL);
    Serial.println();
    // Turn on motor A & B
    digitalWrite(MotFwdL, HIGH); 
    digitalWrite(MotRevL, LOW);
    digitalWrite(MotFwdR, HIGH); 
    digitalWrite(MotRevR, LOW);
    // Serial.print("Forward  ");
    // Serial.println(encoderValueR);
}

//function to make the robot turn left 
//parameter:
//    speed: speed of the robot
void turnLeft(int speed){
  // Reset encoder counter
    lastEncodedL = 0; // Here updated value of encoder store.
    encoderValueL = 0; // Raw encoder value

    lastEncodedR = 0; // Here updated value of encoder store.
    encoderValueR = 0; // Raw encoder value

    // For PWM maximum possible values are 0 to 255
    analogWrite(enR, speed);

    int motor_L_speed = TURN_WEIGHT*speed + K * (TURN_WEIGHT*encoderValueL - encoderValueR);
    analogWrite(enL, motor_L_speed);

    // Turn on motor A & B
    digitalWrite(MotFwdL, HIGH); 
    digitalWrite(MotRevL, LOW);
    digitalWrite(MotFwdR, HIGH); 
    digitalWrite(MotRevR, LOW);
    Serial.print("Left  ");
    Serial.println(encoderValueR);  

}

//function to make the robot turn right 
//parameter:
//    speed: speed of the robot
void turnRight(int speed){
  // Reset encoder counter
    lastEncodedL = 0; // Here updated value of encoder store.
    encoderValueL = 0; // Raw encoder value

    lastEncodedR = 0; // Here updated value of encoder store.
    encoderValueR = 0; // Raw encoder value

    // For PWM maximum possible values are 0 to 255
    analogWrite(enR, speed);

    int motor_L_speed = TURN_WEIGHT*speed + K * (TURN_WEIGHT*encoderValueR - encoderValueL);
    analogWrite(enL, motor_L_speed);

    // Turn on motor A & B
    digitalWrite(MotFwdL, HIGH); 
    digitalWrite(MotRevL, LOW);
    digitalWrite(MotFwdR, HIGH); 
    digitalWrite(MotRevR, LOW);
    Serial.print("Right  ");
    Serial.println(encoderValueR);  

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

void pid_calc() 
{
  
  proportional=Position-set_point;
  integral = integral + proportional; 
  derivative = proportional - last_proportional; 
  last_proportional = proportional;    
  error_value = int(proportional * Kp + integral * Ki + derivative * Kd); 
}
void calc_turn() 
{  
                                              //Restricting the error value between +256. 
if (error_value< -256)     
  { 
error_value = -256; 
  }  
if (error_value> 256) 
  { 
error_value = 256; 
  }  
                                              // If error_value is less than zero calculate right turn speed values 
if (error_value< 0) 
  { 
right_speed = max_speed + error_value; 
left_speed = max_speed; 
  } 
  else 
  { 
right_speed = max_speed; 
left_speed = max_speed - error_value; 
  } 
}

void motor_drive (int right_speed,int left_speed)
{
  if (right_speed>255)
      right_speed=255;
  if (right_speed<0)
      right_speed=0;
  if (left_speed>255)
      left_speed=255;
  if (left_speed<0)
      left_speed=0;
   
  digitalWrite(MotFwdL,1);
  digitalWrite(MotFwdR,1);
  digitalWrite(MotRevL,0);
  digitalWrite(MotRevR,0);
  analogWrite(enL,left_speed);
  analogWrite(enR,right_speed);
  delay(100);
}
