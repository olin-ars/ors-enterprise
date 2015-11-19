#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

std_msgs::Float32 Rudder_msg;
ros::Publisher rudder_pub("RC_rudder_in", &Rudder_msg);
std_msgs::Bool Switch_msg;
ros::Publisher switch_pub("RC_switch_in", &Switch_msg);
std_msgs::Float32 Sail_msg;
ros::Publisher sail_pub("RC_sails_in", &Sail_msg);
std_msgs::Float32 pot_msg;
ros::Publisher pot_pub("potentiometer", &pot_msg);

#define potpin A7

int pinSails = 2;
int pinRudder = 3;
int pinSwitch = 4;

float SailCommand, RudderCommand, Switch;

void setupROS(){
  nh.initNode();
  nh.advertise(rudder_pub);
  nh.advertise(switch_pub);
  nh.advertise(sail_pub);
  nh.advertise(pot_pub);
}

void setup() {
  setupROS();
  pinMode(pinSails, INPUT);
  pinMode(pinRudder, INPUT);
  pinMode(pinSwitch, INPUT);
}

float readPWM(int pin, int range[2]){
  // Timeout on pulsein is 50ms
  int val = pulseIn(pin, HIGH, 50*1000); //NOTE: pulseIn() has pretty nasty resolution sometimes.
  if (val == 0){
    // No pulse recieved, radio probably disconnected
    return 0.0;
  }
  float mid = (range[0] + range[1]) / 2.0;
  float rng = (range[1] - range[0]) / 2.0;
  return (val-mid) / rng;
}

float readPot(){
  return (analogRead(potpin) - 512)/1024.0 * 270.0;  
}

void loop() {
  int SailRange[] = {1150, 1700};
  SailCommand = readPWM(pinSails, SailRange);
  int RudderRange[] = {1180, 1730};
  RudderCommand = readPWM(pinRudder, RudderRange);
  int SwitchRange[] = {1180, 1720};
  Switch = readPWM(pinSwitch, SwitchRange);

  Sail_msg.data = SailCommand;
  sail_pub.publish(&Sail_msg);
  Rudder_msg.data = RudderCommand;
  rudder_pub.publish(&Rudder_msg);
  Switch_msg.data = Switch < 0;
  switch_pub.publish(&Switch_msg);

  pot_msg.data = readPot();
  pot_pub.publish(&pot_msg);

  nh.spinOnce();
  delay(20);
}
