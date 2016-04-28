/*
* ORS demonstration of closed-loop motor control
* When run on a teensy connected to a computer with rosserial,
* this code accepts commands from the topic rudderCommands and 
* initiates a motion of the connected motor to the setpoint specified
* by the command. Feedback is provided through an analog pot_pub,
* and all work is done in degrees -180 -- 180.
*/

#include <ros.h>
#include <std_msgs/Int16.h>

#include <Servo.h> 

#define SERVO_PIN 9
#define potpin A7

ros::NodeHandle  nh;

Servo myservo;  // create servo object to control a servo 

int currentPos;    // variable to read the value from the analog pin 

const int SERVO_CENTER = 92.5;
const float POT_OFFSET = 277;
const int DEADZONE = 5;
int power = 20;

int lastCommanded = -1;
bool newCommand = false;

std_msgs::Int16 pot_msg;
ros::Publisher pot_pub("/rudder/pos", &pot_msg);
std_msgs::Int16 dir_msg;
ros::Publisher dir_pub("/rudder/motor_direction", &dir_msg);

void command_callback(const std_msgs::Int16& command){
	if(command.data <= -180 || command.data >= 180){
		return;
	}
	lastCommanded = command.data;
	newCommand = true;
}
ros::Subscriber<std_msgs::Int16> command_sub("/rudder/set_point", &command_callback);

void power_callback(const std_msgs::Int16& msg){power = msg.data;}
ros::Subscriber<std_msgs::Int16> center_sub("/rudder/powerconstant", &power_callback);

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

	myservo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object 
	myservo.write(SERVO_CENTER);

	// Turn on the light
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
}

int movementDirection = 0; // 0 for stopped, 1 , -1 for current movement direction.

void moveServo(){
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

	if(abs(currentPos - lastCommanded) <= DEADZONE){
		// We have reached our target
		myservo.write(SERVO_CENTER);
		movementDirection = 0;
	}
	dir_msg.data = movementDirection;
}

float readPot(){
	const float gear_ratio = 0.25;
	float reading = (-analogRead(potpin) * (360.0 / 1024)) - POT_OFFSET;
    while (reading < -180){reading += 360;}
    while (reading >= 180){reading -= 360;}

	static int turnsOffset = 0;
	static float lastreading = reading;

	if (reading > lastreading + 180.0){
		turnsOffset--;
	}
	if (reading < lastreading - 180.0){
		turnsOffset++;
	}
	lastreading = reading;

    return (reading + 360*turnsOffset)*gear_ratio;
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
