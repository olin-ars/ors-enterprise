int pins = 5;
int vals;
int mins = 1150;
int maxs = 1700;
int mids = (mins + maxs)/2;
float ranges = (maxs-mins)/2;
float s;

int piny = 3;
int valy;
int miny = 1180;
int maxy = 1720;
int midy = (miny + maxy)/2;
float rangey = (maxy-miny)/2;
float y;

int pinx = 4;
int valx;
int minx = 1180;
int maxx = 1730;
int midx = (minx + maxx)/2;
float rangex = -(maxx-minx)/2;
float x;

void setup() {
  pinMode(pins, INPUT);
  pinMode(pinx, INPUT);
  pinMode(piny, INPUT);
  Serial.begin(9600);

}

void loop() {
  vals = pulseIn(pins, HIGH);
  s = (vals-mids)/ranges;
  valx = pulseIn(pinx, HIGH);
  x = (valx-midx)/rangex;
  valy = pulseIn(piny, HIGH);
  y = (valy-midy)/rangey;
  //Serial.println(valy);
  Serial.println(s);
  Serial.println(x);
  Serial.println(y);
  delay(100);
}
