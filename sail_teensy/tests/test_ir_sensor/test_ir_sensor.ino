int ir_pin = A0;
float val;
float voltage;

void setup(){
  pinMode(ir_pin, INPUT);
  Serial.begin(9600);
}

void loop(){
  val = analogRead(ir_pin);
  voltage = val*3.3/1024;
  Serial.println(voltage);
  delay(100);
}