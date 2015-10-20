#define arrayLength 7                        // used everywhere except line 14, I couldn;t figure out how to make that line better
#define raAlpha 0.1                          //used for running average to weight current data and average data
#define triggerVal 1.15                      //used as margin that sensor val must be over in order to count for sensor activating
int hallVals[]     = {0, 0, 0, 0, 0, 0, 0};  //used to store which hall effect sensors are currently active
float hallAvgs[]   = {0, 0, 0, 0, 0, 0, 0};  //used to store running averages of past hall sensor data
int lastHallVals[] = {0, 0, 0, 0, 0, 0, 0};  //used to store which hall was last active
int dir = -1;                                //used to store the linear actuator direction


void setup() {
  Serial.begin(9600);
  // Instantiate averages
  for(int i = 0; i < arrayLength; i++){
    if (hallAvgs[i] == 0) {
      hallAvgs[i] = analogRead(i);
    }
  }
}


void loop() {
  readHalls();
  printHallAndDirection();
  
  if(hallVals[0] !=0 || hallVals[1] !=0 || hallVals[2] !=0 || hallVals[3] !=0 || hallVals[4] !=0 || hallVals[5] !=0 || hallVals[6] !=0){
    //this if is to check if there is an active hall sensor, if there isn't then we don't want to change the array holding the last active one
    for(int i = 0; i < arrayLength; i++){
      // This used to be memcpy, but that erased the hallVals values, which caused logic problems
      lastHallVals[i] = hallVals[i];
    }
  }
  delay(50);
}


void readHalls(){ //reads all the hall sensors, if any are active (giving a value of more than 800) it stores a 1 in that sensors position in the active array
  for(int i = 0; i < arrayLength; i++){
    int readVal = analogRead(i);
    if(readVal-hallAvgs[i] >= triggerVal){
      hallVals[i] = 1;
    } else if(readVal-hallAvgs[i] <= -triggerVal){
      hallVals[i] = 0;
    }
    hallAvgs[i] = ((1-raAlpha)*hallAvgs[i])+(raAlpha*readVal);
  }
  Serial.println("");
}


// Looks through the active array and says which is active. if none are active it returns -1
int returnActiveHall(){
  int activeHall = -1;
  for(int i = 0; i < arrayLength; i++){
    if(hallVals[i] == 1){
      activeHall = i;
    }
  }
  return activeHall;
}


// Same as returnActiveHall() but looks through the last active array instead
int returnLastActiveHall(){
  int lastActiveHall = -1;
  for(int i = 0; i < arrayLength; i++){
    if(lastHallVals[i] == 1){
      lastActiveHall = i;
    }
  }
  return lastActiveHall;
}


// Will return 1 if moving forward, 0 if moving backwards, -1 if no direction yet established
int returnDirection(){
  if(returnActiveHall() > -1){
    if (returnActiveHall() > returnLastActiveHall()){
      dir = 1;
    }
    else if (returnActiveHall() < returnLastActiveHall()){
      dir = -1;
    }
  }
  return dir;
}


// If there's and active hall, this prints which is active and the direction. If none are active it prints the direction.
void printHallAndDirection(){
  Serial.println("");
  Serial.print("Hall Vals:");
  for (int i=0; i<arrayLength; i++) {
    Serial.print(hallVals[i]);
    Serial.print("\t\t");
  }
  Serial.println("");

  if(returnActiveHall() > -1){
    Serial.print("Hall Sensor ");
    Serial.print(returnActiveHall());
    Serial.print(" is activated, Direction is ");
    Serial.println(returnDirection());
  }
  else{
    Serial.print("No hall sensor is currently active, direction is ");
    Serial.println(returnDirection());
  }
}

