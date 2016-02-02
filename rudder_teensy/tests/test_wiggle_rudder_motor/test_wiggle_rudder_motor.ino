#include <Servo.h> 
 
int ledPin = 13;
int ledVal = 0;
Servo myservo;
int velocity = 0;
 
void setup() 
{ 
  pinMode(ledPin, OUTPUT);
  myservo.attach(9);
  Serial.begin(9600);
} 
 
void loop() 
{
  for(velocity = 92.5; velocity <= 105; velocity += 1)
  {
    myservo.write(velocity);
    delay(15);
  }
  digitalWrite(ledPin, ledVal);
  ledVal = !ledVal;

  for(velocity = 105; velocity>=80; velocity-=1)
  {                                
    myservo.write(velocity);
    delay(15);
  }
  digitalWrite(ledPin, ledVal);
  ledVal = !ledVal;

  for(velocity = 80; velocity <= 92.5; velocity += 1)
  {
    myservo.write(velocity);
    delay(15);
  }
  digitalWrite(ledPin, ledVal);
  ledVal = !ledVal;
  delay(500);
} 
