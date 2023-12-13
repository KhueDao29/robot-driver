String sign = "";
void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("c");
    
  while ((sign[0] != 'l') && (sign[0] != 'r') && (sign[0] != 's') && (sign[0] != 'a'))
  {
    if (Serial.available() > 0) {
      sign = Serial.readStringUntil('\n');
      // Serial.println("k");
      // delay(100);
      Serial.println(sign);
    }
  }
  delay(2000);
  sign = "";
  // Serial.println(sign);
}