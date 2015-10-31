#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 x_msg;
ros::Publisher x_pub("RC_x_axis", &x_msg);
std_msgs::Float32 y_msg;
ros::Publisher y_pub("RC_y_axis", &y_msg);
std_msgs::Float32 s_msg;
ros::Publisher s_pub("RC_s_axis", &s_msg);

int pins = 5;
int piny = 3;
int pinx = 4;

float s, x, y;

void setupROS(){
  nh.initNode();
  nh.advertise(x_pub);
  nh.advertise(y_pub);
  nh.advertise(s_pub);
}

void setup() {
  setupROS();
  pinMode(pins, INPUT);
  pinMode(pinx, INPUT);
  pinMode(piny, INPUT);
}

float readPWM(int pin, int range[2]){
	int val = pulseIn(pin, HIGH); //NOTE: pulseIn() has pretty nasty resolution sometimes.
  if (val == 0){
    // No pulse recieved, radio probably disconnected
    return 0.0;
  }
	float mid = (range[0] + range[1]) / 2.0;
	float rng = (range[1] - range[0]) / 2.0;
  return (val-mid) / rng;
}

void loop() {
  int srange[] = {1150, 1700};
  s = readPWM(pins, srange);
  int xrange[] = {1180, 1730};
  x = readPWM(pinx, xrange);
  int yrange[] = {1180, 1720};
  y = readPWM(piny, yrange);

  s_msg.data = s;
  s_pub.publish(&s_msg);
  x_msg.data = x;
  x_pub.publish(&x_msg);
  y_msg.data = y;
  y_pub.publish(&y_msg);

  nh.spinOnce();
  delay(20);
}
