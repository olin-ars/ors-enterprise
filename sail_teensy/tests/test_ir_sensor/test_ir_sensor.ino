int ir_pin = A0;
float val;
float voltage;

void setup(){
  pinMode(ir_pin, INPUT);
  Serial.begin(9600);
}

void loop(){
  voltage = analogRead(ir_pin) * 3.3 / 1024;
  Serial.println(voltage);
  delay(100);
}