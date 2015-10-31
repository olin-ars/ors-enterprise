int pins = 5;
int piny = 3;
int pinx = 4;

float s, x, y;

void setup() {
  pinMode(pins, INPUT);
  pinMode(pinx, INPUT);
  pinMode(piny, INPUT);
  Serial.begin(9600);

}

float readPWM(int pin, int range[2]){
	int val = pulseIn(pin, HIGH); //NOTE: pulseIn() has pretty nasty resolution sometimes.
	float mid = (range[0] + range[1]) / 2.0;
	float rng = (range[1] - range[0]) / 2.0;
  return (val-(mid)) / rng;
}

void loop() {
  int srange[] = {1150, 1700};
  s = readPWM(pins, srange);
  int xrange[] = {1180, 1730};
  x = readPWM(pinx, xrange);
  int yrange[] = {1180, 1720};
  y = readPWM(piny, yrange);
  //Serial.println(valy);
  Serial.println(s);
  Serial.println(x);
  Serial.println(y);
  delay(100);
}
