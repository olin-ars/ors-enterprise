/*
* ORS demonstration of closed-loop motor control
* When run on a teensy connected to a computer with rosserial,
* this code accepts commands from the topic rudderCommands and 
* initiates a motion of the connected motor to the setpoint specified
* by the command. Feedback is provided through an series of magnetic switches,
* and all work is done in counts of magnetic switches.
*/

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include <Servo.h> 

#define SERVO_PIN 9

#define DEADZONE 0

#define FIRST_SENSOR_PIN 14
#define NUM_SENSORS 7


ros::NodeHandle  nh;

Servo myservo;  // create servo object to control a servo 

float currentPos = -1;

int SERVO_CENTER = 92;
float lastCommanded = -1;
bool newCommand = false;

std_msgs::Float32 pos_msg;  // Message from 0 to NUM_SENSORS giving the current sail location
ros::Publisher pos_pub("/sail/pos", &pos_msg);
std_msgs::Int16 dir_msg;
ros::Publisher dir_pub("/sail/motor_direction", &dir_msg);

void command_callback(const std_msgs::Float32& command){
    if(command.data < 0 || command.data > (NUM_SENSORS - 1)){
        return;
    }
    lastCommanded = command.data;
    newCommand = true;
}
ros::Subscriber<std_msgs::Float32> command_sub("/sail/set_point", &command_callback);

void center_callback(const std_msgs::Int16& msg){SERVO_CENTER = msg.data;}
// DEPRECIATED
ros::Subscriber<std_msgs::Int16> center_sub("/sail/ServoCenter", &center_callback);

void setupROS(){
    nh.initNode();
    nh.advertise(pos_pub);
    nh.advertise(dir_pub);
    nh.subscribe(command_sub);
    nh.subscribe(center_sub);
}

void setup()
{
    setupROS();

    myservo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object 
    myservo.write(SERVO_CENTER);

    for(int i = 0; i < NUM_SENSORS; i++){
        pinMode(FIRST_SENSOR_PIN + i, INPUT_PULLUP);
    }

    //Turn on light

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
}

int movementDirection = 0; // 0 for stopped, 1 , -1 for current movement direction.

void moveMotor(){
    const int power = 20;

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

float readSensors(){
    int total = 0;
    int count = 0;
    for(int i = 0; i < NUM_SENSORS; i++){
        bool switchVal = !digitalRead(FIRST_SENSOR_PIN + i);
        if (switchVal){
            total += i;
            count++;
        }
    }
    if (count == 0) 
        return currentPos;
    float newPos = float(total)/count;
    return newPos;
    // Note that this function sorta-expects that the output it returns will go into
    // a global variable called currentPos.
}

// Controls the frequency with which ROS transmits/recieves data.
// This allows the control loop to run much faster while still attempting
// to recieve / transmit updates in a timely way.
unsigned int ros_transmit_period = 25; //milliseconds

void loop()
{
    currentPos = readSensors();    // reads the position of the motor
    pos_msg.data = currentPos;

    static unsigned long last_ros_transmit = millis();
    if (millis() - last_ros_transmit >= ros_transmit_period){
        last_ros_transmit = millis();

        dir_pub.publish( &dir_msg );
        pos_pub.publish( &pos_msg );
        nh.spinOnce();
    }

    moveMotor();
    delay(1);
}
