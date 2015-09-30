const int encoderPin = A0;          // input pin for the potentiometer
int rawEncoderVal = 0;              // raw analog read value
const int rawEncoderValToRadians = 2 * 3.14159 / 1024.0;
float encoderAngleRadians = 0.0;    // tracks current rudder angle
float encoderAngleZeroOffset = 0.0; // we'll want to set the "zerod" position of the rudder

void setup() {
  Serial.begin(9600);
  pinMode(encoderPin, INPUT);
}

void loop() {
    // Grab encoder values and calculate position in radians
    rawEncoderVal = analogRead(encoderPin);
    encoderAngleRadians = rawEncoderVal * rawEncoderValToRadians;

    // Print useful debug information
    Serial.println("Debug information:");
    Serial.print("Raw rudder value: ");
    Serial.println(rawEncoderVal);
    Serial.print("Rudder angle value: ");
    Serial.println(encoderAngleRadians);
}
