#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

int sensor1,sensor2;

// set the pins to shutdown
#define SHT_LOX1 7
#define SHT_LOX2 6

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;


void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);

  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  delay(10);
}


void read_dual_sensors() {
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  if(measure1.RangeStatus != 4) {     // if not out of range
    sensor1 = measure1.RangeMilliMeter;     
  } else {
    Serial.print("Out of range");
  }
  
  // print sensor two reading
  if(measure2.RangeStatus != 4) {
    sensor2 = measure2.RangeMilliMeter;
  } else {
    Serial.print("Out of range");
  }
  
  static int i, sensorLinks, sensorRechts = 0;
  sensorLinks += sensor1;
  sensorRechts += sensor2;
  i++;  

  if(i == 10){
    const int sensorsApartFromEachother = 75;   //mm
    int distanceDiff = sensorLinks/i - sensorRechts/i;
    float radian = atan2(distanceDiff, sensorsApartFromEachother);
    float angle = radian * 180/3.14159265;
    Serial.print(sensorLinks/i); Serial.print(" \t ");
    Serial.print(sensorRechts/i); Serial.print(" \t "); Serial.println(angle);
    i, sensorLinks, sensorRechts = 0;
    delay(1000); //to observe the angle
  }
}



void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  Serial.println("Both in reset mode...(pins are low)");
  
  setID();
}


void loop() {
  read_dual_sensors();
}