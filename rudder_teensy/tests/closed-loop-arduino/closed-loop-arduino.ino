/*
 * ORS demonstration of closed-loop motor control
 * When run on a teensy connected to a computer with rosserial,
 * this code accepts commands from the topic rudderCommands and 
 * initiates a motion of the connected motor to the setpoint specified
 * by the command. Feedback is provided through an analog potentiometer,
 * and all work is done in degrees 0-360.
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
const int DEADZONE = 5;
int lastCommanded = -1;
bool newCommand = false;

std_msgs::Int16 pot_msg;
ros::Publisher potentiometer("rudderSensor", &pot_msg);
std_msgs::Int16 dir_msg;
ros::Publisher direction_chn("rudderMotorDirection", &dir_msg);

void commandCB(const std_msgs::Int16& command){
	if(command.data <= 0 || command.data >= 360){
		return;
	}
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
	const int power = 5;

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
	direction_chn.publish( &dir_msg );
}

float readPot(){
	return 360 - analogRead(potpin) * (360.0 / 1024);  
}

void loop()
{
	currentPos = readPot();          // reads the value of the potentiometer (value between 0 and 1023) 

	pot_msg.data = currentPos;
	potentiometer.publish( &pot_msg );

	moveServo();

	nh.spinOnce();
	delay(10);
}
