/*
testing out the RC controllers w/ teensy code!
-matt
*/

/* // Commenting it out to make this a more general test thing.
int rc_position = 0;

void setup() {
    
    pinMode(3, OUTPUT); 
    pinMode(4, INPUT); // 4 takes input from the rc receiver; 3 is the output to the jaguar

    Serial.begin(9600);

}

void loop() {

    rc_position = pulseIn(4, HIGH);
    Serial.println(rc_position);

    //int value_to_write = map(rc_position, )
}
*/

	int port3;
	int port4;
	int port5;
	int port6;
	int port7; //not PWM
	int port9;
	int port10;

void setup() {

	pinMode(3, INPUT);
	pinMode(4, INPUT);
	pinMode(5, INPUT);
	pinMode(6, INPUT);
	pinMode(7, INPUT);
	pinMode(9, INPUT);
	pinMode(10, INPUT);

	Serial.begin(9600);

}

void loop() {

	port3 = pulseIn(3, HIGH);
	port4 = pulseIn(4, HIGH);
	port5 = pulseIn(5, HIGH);
	port6 = pulseIn(6, HIGH);
	port7 = pulseIn(7, HIGH);
	port9 = pulseIn(9, HIGH);
	port10 = pulseIn(10, HIGH);

	Serial.print("Port 3: "); Serial.println(port3);
	Serial.print("Port 4: "); Serial.println(port4);
	Serial.print("Port 5: "); Serial.println(port5);
	Serial.print("Port 6: "); Serial.println(port6);
	Serial.print("Port 7: "); Serial.println(port7);
	Serial.print("Port 9: "); Serial.println(port9);
	Serial.print("Port 10: "); Serial.println(port10);

}


/*
Pulses to Parameters:
Throttle (left vertical) goes from 1125 (hard bot) to 1896 (hard top), resting where left.
Aile (right horizontal) goes from 1890 at hard left to 1092 (hard right), resting at 1489.
Elev (right vertical) goes from 1117 (hard bot) to 1920 (hard top), resting at 1519.
Rudd (left horizontal) goes from 1920 (hard left) to 1117 (hard right), resting at 1519.
Gear (upper left corner switch; higher one) switches from 1897 (down) to 1596 (middle) to 1094 (top)
Aux 1 (the bind switch) rests at 1095, and when raised is at 1897.

All values experience 1-10 points of noise. Values obtained using Spektrum DX5e transmitter, Spektrum AR610-X receiver; Teensy connected via USB; builtin Teensy power.
*/


