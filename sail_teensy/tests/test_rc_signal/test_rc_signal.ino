/*
testing out the RC controllers w/ teensy code!
-matt
*/

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