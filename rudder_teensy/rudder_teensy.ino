/*
* ORS demonstration of closed-loop motor control
* When run on a teensy connected to a computer with rosserial,
* this code accepts commands from the topic rudderCommands and 
* initiates a motion of the connected motor to the setpoint specified
* by the command. Feedback is provided through an analog pot_pub,
* and all work is done in degrees 0-360.
*/

/*
    Electrical setup:
    Outputs:
      Pin SERVO_PIN [D9] connected to PWM input of motor controller, with value of 88
      referring to center.

    Inputs:
      Pin potpin [A7] connected to potentiometer. Analog input expected in range 0..3V
      Pins STAR_LIM_PIN and PORT_LIM_PIN [11], [12] connected through limit switches.
        The other wire of the limit switch connects to ground.
        The limit switch is expected to be normally closed, and the Teensy provides
        an internal pullup resistor.
*/

#include <ros.h>
#include <std_msgs/Int16.h>

#include <Servo.h> 

#define SERVO_PIN 9
#define potpin A7

#define STAR_LIM_PIN 11 //StarBoard limit switch pin
#define PORT_LIM_PIN 12 //Port limit switch pin
//Assumption : STAR = (numerically) bigger angle, PORT = less angle

int softLimStop = 0;

ros::NodeHandle nh;

Servo myservo;  // create servo object to control a servo 

int currentPos;    // variable to read the value from the analog pin 

int SERVO_CENTER = 88;
const int DEADZONE = 5;
int lastCommanded = -1;
bool newCommand = false;

std_msgs::Int16 pot_msg;
ros::Publisher pot_pub("rudderSensor", &pot_msg);
std_msgs::Int16 dir_msg;
ros::Publisher dir_pub("rudderMotorDirection", &dir_msg);

void command_callback(const std_msgs::Int16& command){
  if(command.data <= 0 || command.data >= 360){
    return;
  }
  lastCommanded = command.data;
  newCommand = true;
}
ros::Subscriber<std_msgs::Int16> command_sub("rudderCommands", &command_callback);

void center_callback(const std_msgs::Int16& msg){SERVO_CENTER = msg.data;}
ros::Subscriber<std_msgs::Int16> center_sub("rudderServoCenter", &center_callback);

void setupROS(){
  nh.initNode();
  nh.advertise(pot_pub);
  nh.advertise(dir_pub);
  nh.subscribe(command_sub);
  nh.subscribe(center_sub);
}

void setup()
{
  setupROS();
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object 
  myservo.write(SERVO_CENTER);
  pinMode(STAR_LIM_PIN,INPUT_PULLUP);
  pinMode(PORT_LIM_PIN,INPUT_PULLUP);
  pinMode(13,OUTPUT); //Check LED Activate
  digitalWrite(13,HIGH);
}

int movementDirection = 0; // 0 for stopped, 1 , -1 for current movement direction.

void moveServo(){
  const int power = 5;

    
  // A valid command has just been recieved
  if(newCommand){
    newCommand = false;
    if (lastCommanded > currentPos){
      movementDirection = 1;
    }
    else{
      movementDirection = -1;
    }
  }  
  if(abs(currentPos - lastCommanded) <= DEADZONE){
    // We have reached our target
    movementDirection = 0;
  }

  // DEAL with limit switches
  
  if(digitalRead(STAR_LIM_PIN) == HIGH && movementDirection > 0){
    movementDirection = 0;
  }  
  if(digitalRead(PORT_LIM_PIN) == HIGH && movementDirection < 0){
    movementDirection = 0;
  }

  // WRITE motor values

  myservo.write(SERVO_CENTER + power*movementDirection);

  dir_msg.data = movementDirection;
}

float readPot(){
  return 360 - analogRead(potpin) * (360.0 / 1024);  
}

// Controls the frequency with which ROS transmits/recieves data.
// This allows the control loop to run much faster while still attempting
// to recieve / transmit updates in a timely way.
unsigned int ros_transmit_period = 10; //milliseconds

void loop()
{
  currentPos = readPot();          // reads the value of the pot_pub (value between 0 and 1023) 
  pot_msg.data = currentPos;

  static unsigned long last_ros_transmit = millis();
  if (millis() - last_ros_transmit >= ros_transmit_period){
    last_ros_transmit = millis();

    dir_pub.publish( &dir_msg );
    pot_pub.publish( &pot_msg );
    nh.spinOnce();
  }
    
  moveServo();

  delay(1);
}
