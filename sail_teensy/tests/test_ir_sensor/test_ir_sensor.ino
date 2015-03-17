int ir_pin = A0;
float val;
float voltage;

// Data for the 3Y Sharp IRs (see Rocco/Eric for details)
const int IR_DATA_LENGTH = 12;
int ir_distance[IR_DATA_LENGTH] = {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200};
float ir_voltage[IR_DATA_LENGTH] = {2.63, 2.57, 1.94, 1.49, 1.18, 0.99, 0.85, 0.71, 0.61, 0.55, 0.49, 0.45};

void setup(){
  pinMode(ir_pin, INPUT);
  Serial.begin(9600);
}

void loop(){
  voltage = analogRead(ir_pin) * 3.3 / 1024;
  Serial.print("Voltage: "); Serial.println(voltage);
  Serial.print("Distance (mm): ");
  Serial.println(sharpIRVoltageToMillimeters(voltage));
  delay(100);
}

// Takes a Sharp IR voltage value and converts that reading to
// millimeters (+- 10 mm)
int sharpIRVoltageToMillimeters(float voltage) {
    int index = 0;
    int index_found = 0;
    int i;
    for (i = 0; i < IR_DATA_LENGTH; i++) {
        if (ir_voltage[i] < voltage && !index_found) {
            index_found = 1;
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

