/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

 #include <ros.h>
 #include <std_msgs/Int16.h>

 #include <Servo.h> 

 #define SERVO_PIN 9
 #define potpin A7

 ros::NodeHandle  nh;

Servo myservo;  // create servo object to control a servo 

int currentPos;    // variable to read the value from the analog pin 

int SERVO_CENTER = 86;
int lastCommanded = -1;
bool newCommand = false;

std_msgs::Int16 pot_msg;
ros::Publisher potentiometer("rudderSensor", &pot_msg);
std_msgs::Int16 dir_msg;
ros::Publisher direction_chn("rudderMotorDirection", &dir_msg);

void commandCB(const std_msgs::Int16& command){
    lastCommanded = command.data;
    newCommand = true;
}
ros::Subscriber<std_msgs::Int16> commands("rudderCommands", &commandCB);


void setup()
{
  nh.initNode();
  nh.advertise(potentiometer);
  nh.advertise(direction_chn);
  nh.subscribe(commands);

  myservo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object 
  myservo.write(SERVO_CENTER);
}

int movementDirection = 0; // 0 for stopped, 1 , -1 for current movement direction.

void moveServo(){
    const int power = 3;

    if (newCommand){
        // A command has just been recieved
        newCommand = false;
        if (lastCommanded > currentPos){
            myservo.write(SERVO_CENTER + power);
            movementDirection = 1;
        }
        else{
            myservo.write(SERVO_CENTER - power);
            movementDirection = -1;
        }
    }

    if(
        ((movementDirection == 1) && currentPos >= lastCommanded) ||
        ((movementDirection == -1) && currentPos <= lastCommanded)){
        // We have reached our target
        myservo.write(SERVO_CENTER);
        movementDirection = 0;
    }
    dir_msg.data = movementDirection;
    direction_chn.publish( &dir_msg );
}

void loop()
{
  currentPos = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 

  pot_msg.data = currentPos;
  potentiometer.publish( &pot_msg );

  moveServo();

  nh.spinOnce();
  delay(10);
}
