/*
File contains functions/etc to interpret RC input.
Currently, just has the switchOn and within functions.
-Matt
*/

#include <Servo.h> 

#define IR_PIN_IN           A0
#define RC_SIGNAL_PIN_IN    3
#define JAG_PWM__PIN_OUT    4
#define RC_MAX_SIGNAL       1950  // Result of pulseIn on PWM
#define RC_MID_SIGNAL       1500  // Result of pulseIn on PWM
#define RC_MIN_SIGNAL       1100  // Result of pulseIn on PWM
#define MIN_POSITION        200  // Millimeters
#define MAX_POSITION        400  // Millimeters

Servo jaguar;

// Data for the 3Y Sharp IRs (see Rocco/Eric for details)
const int IR_DATA_LENGTH = 12;
int ir_distance[IR_DATA_LENGTH] = {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200}
float ir_voltage[IR_DATA_LENGTH] = {2.63, 2.57, 1.94, 1.49, 1.18, 0.99, 0.85, 0.71, 0.61, 0.55, 0.49, 0.45};


int distance_to_motor;
int rc_signal;
int goal_distance;


void setup(){
    pinMode(IR_PIN_IN, INPUT);
    pinMode(RC_SIGNAL_PIN_IN, INPUT);
    jaguar.attach(JAG_PWM__PIN_OUT);
    Serial.begin(9600);
}


void loop(){
    rc_signal = pulseIn(RC_SIGNAL_PIN_IN, HIGH);
    distance_to_motor = getActuatorPosition();
    goal_distance = map(rc_signal, RC_MIN_SIGNAL, RC_MAX_SIGNAL,
                        MIN_POSITION, MAX_POSITION);

    if (abs(distance_to_motor - goal_distance) < 50) {
        Serial.println("Close enough!");
        jaguar.write(90);
    } else if (distance_to_motor < goal_distance) {
        Serial.println("Not far enough, moving out!");
        jaguar.write(100);
    } else if (distance_to_motor > goal_distance) {
        Serial.println("Too far, moving back");
        jaguar.write(80);
    }

    delay(10);
}


// Gets the actuator position in mm
int getActuatorPosition() {
    float voltage = analogRead(IR_PIN_IN) * 3.3 / 1024;
    return sharpIRVoltageToMillimeters(voltage);
}

// Takes a Sharp IR voltage value and converts that reading to
// millimeters (+- 10 mm)
int sharpIRVoltageToMillimeters(float voltage) {
    int i;
    int index = 0;
    int index_found = 0;
    for (i = 0; i < IR_DATA_LENGTH; i++) {
        if (ir_voltage[i] < voltage && !index_found) {
            int index_found = 1;
            if (i == 0) {
                index = 0;
            } else {
                index = i - 1;
            }
        }
    }
    float local_slope = (ir_voltage[index + 1] - ir_voltage[index]) /
                        (ir_distance[index + 1] - ir_distance[index]);
    float delta_voltage = voltage - ir_voltage[index];
    int delta_distance = delta_voltage / local_slope;
    return (ir_distance[index] + delta_distance);
}

bool switchOn(int switchPin) {
//Returns true if the pulse_length of the switchPin is greater than predefined RC_MID_SIGNAL.
    int pulse_length = pulseIn(switchPin, HIGH);
    if (pulse_length > RC_MID_SIGNAL) {
        return true;
    }
    else {
        return false;
    }
}

bool within(int value, int band_center, int band_radius) {
//Returns true if value is within band_radius of band_center; false otherwise.
    if ((band_center + band_radius > value) && (band_center - band_radius < value)) {
        return true;
    }
    else {
        return false;
    }
}
