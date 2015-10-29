#define arrayLength 7 //used everywhere except line 14, I couldn;t figure out how to make that line better
#define firstPin 14
float pos = 0; //used to store where the magnet probably is.

int dir = -1; //used to store the linear actuator direction

void setup() {
  Serial.begin(9600);
  for(int i = 0; i < arrayLength; i++){
    pinMode(i+firstPin, INPUT_PULLUP);
  }
}

void loop() {
  readSwitches();
  printSwitchAndDirection();
  
  delay(5);
}

void readSwitches(){ //reads all the switch sensors, if any are active it stores a 1 in that sensors position in the active array
  int total;
  int count;
  for(int i = 0; i < arrayLength; i++){
    bool switchVal = !digitalRead(firstPin+i);
    if (switchVal){
      total += i;
      count++;
    }
  }
  if (count == 0) return;
  float newPos = float(total)/count;
  if (newPos != pos){
    if (newPos > pos) dir = 1;
    else dir = 0;
  }
  pos = newPos;
}

void printSwitchAndDirection(){ //if there's and active switch, this prints which is active and the direction. If none are active it prints the direction.
  Serial.print("Switch ");
  Serial.print(pos);
  Serial.print(" is activated, Direction is ");
  Serial.println(dir);
}
