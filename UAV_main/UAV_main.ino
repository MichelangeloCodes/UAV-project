#include "Adafruit_VL53L0X.h"

//DEFINE PINS
#define start_button 2
#define emergency_button 3  //external interrupt
#define SHT_LOX1 5
#define SHT_LOX2 6
#define SHT_LOX3 7
#define motorTangial 46
#define steerEngineLeft 11
#define steerEngineRight 12
#define blowers 13

// ADRESS TO ASSIGN TO VL530X
#define VL530x_ADDRESS_left  0x30
#define VL530x_ADDRESS_right 0x31
#define VL530x_ADDRESS_side  0x32
int sensor1,sensor2,sensor3;

// objects for the vl53l0x
Adafruit_VL53L0X TOF1_left = Adafruit_VL53L0X();
Adafruit_VL53L0X TOF2_right = Adafruit_VL53L0X();
Adafruit_VL53L0X TOF3_side = Adafruit_VL53L0X();

//holds measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

//VARIABLE  
volatile bool emergencyState = false;
bool lastButtonState = LOW;
bool blowerState = LOW;
int pwmValue1 = 0;
int pwmValue2 = 0;
int pwmValue3 = 100;



void setup() {
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(start_button, INPUT_PULLUP);
  pinMode(emergency_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(emergency_button), myISR, FALLING);
  
  pinMode(blowers, OUTPUT);
  pinMode(steerEngineLeft, OUTPUT);
  pinMode(steerEngineRight, OUTPUT);

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  Serial.println("Both in reset mode...(pins are low)");
  
  Serial.println("Starting...");
  setID();
  initalisation();
}



void loop() {
  if(emergencyState == false){
    byte buttonState = digitalRead(start_button);
    if (buttonState != lastButtonState) {
      lastButtonState = buttonState;
      if (buttonState == LOW) {
        blowerState = (blowerState == HIGH) ? LOW: HIGH;
        digitalWrite(blowers, blowerState);
      }
    }
  }
  if(emergencyState == true){
    digitalWrite(blowers, LOW);
  }
   
  read_dual_sensors();
  steering();
}



void myISR()
{
  Serial.println("trigger");
  emergencyState = !emergencyState;
}



void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if(!TOF1_left.begin(VL530x_ADDRESS_left)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!TOF2_right.begin(VL530x_ADDRESS_right)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if(!TOF3_side.begin(VL530x_ADDRESS_side)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }
}



void initalisation()
{
  //for initialsation
}



void read_dual_sensors() {
  TOF1_left.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  TOF2_right.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  TOF3_side.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!

  Serial.print("1: ");
  if(measure1.RangeStatus != 4) {     // if not out of range
    sensor1 = measure1.RangeMilliMeter;    
    Serial.print(sensor1);
    Serial.print("mm");    
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" ");

  Serial.print("2: ");
  if(measure2.RangeStatus != 4) {
    sensor2 = measure2.RangeMilliMeter;
    Serial.print(sensor2);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }

  Serial.print(" ");

  Serial.print("3: ");
  if(measure3.RangeStatus != 4) {
    sensor3 = measure3.RangeMilliMeter;
    Serial.print(sensor3);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();
}

void steering()
{
  leftMotor();
  rightMotor();
  sideMotor();

}
void leftMotor(){
  analogWrite(steerEngineLeft, pwmValue1);
}
void rightMotor(){
  analogWrite(steerEngineRight, pwmValue2);
}
void sideMotor(){
  analogWrite(motorTangial, pwmValue3);
}
