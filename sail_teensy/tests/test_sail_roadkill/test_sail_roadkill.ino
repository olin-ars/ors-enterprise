#include <Servo.h> 
 
int ledPin = 13;
int ledVal = 0;
Servo myservo;
int counter = 0;
int servo_val = 92;
 
void setup() 
{ 
  pinMode(ledPin, OUTPUT);
  myservo.attach(9);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  Serial.begin(9600);
} 
 
void loop() 
{
  Serial.print("Sensors: "); Serial.print(digitalRead(14));
  Serial.print(", "); Serial.print(digitalRead(15));
  Serial.print(", "); Serial.print(digitalRead(16));
  Serial.print(", "); Serial.print(digitalRead(17));
  Serial.print(", "); Serial.print(digitalRead(18));
  Serial.print(", "); Serial.print(digitalRead(19));
  Serial.print(", "); Serial.println(digitalRead(20));
  
  counter ++;
  delay(25);
  
  if (counter == 200) {
    counter = 0;
    if (servo_val > 92) { servo_val = 60; }
    else { servo_val = 125; }
    myservo.write(servo_val);
    digitalWrite(ledPin, ledVal);
    ledVal = !ledVal;
  }
} 
