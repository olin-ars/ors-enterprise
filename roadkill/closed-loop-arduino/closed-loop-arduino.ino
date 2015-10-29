/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Int16.h>

#include <Servo.h> 

ros::NodeHandle  nh;

std_msgs::Int16 pot_msg;
ros::Publisher potentiometer("potentiometer", &pot_msg);

char hello[13] = "hello world!";

int i = 0;

#define SERVO_PIN 9

Servo myservo;  // create servo object to control a servo 
 
int potpin = A7;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 

void setup()
{
  nh.initNode();
  nh.advertise(potentiometer);

  myservo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object 
}

void loop()
{
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 

  i++;
  pot_msg.data = val;
  potentiometer.publish( &pot_msg );
  nh.spinOnce();
  delay(100);
}
