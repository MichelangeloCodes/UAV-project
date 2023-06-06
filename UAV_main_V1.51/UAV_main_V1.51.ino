#include "Adafruit_VL53L0X.h"

int serialAllowmend = 1;

//DEFINE PINS
#define RelaisBlower 22
#define RelaisSysteem 23
#define Maxon_ALinks 4
#define Maxon_ARechts 5
#define Maxon_BLinks 6
#define Maxon_BRechts 7
#define Farnell_ALinks 8
#define Farnell_ARechts 9
#define SHT_LOX1 39
#define SHT_LOX2 41
#define SHT_LOX3 43
#define ledBlauw 50 
#define ledRood 51
#define sensVoltCel1 A5
#define sensVoltCel2 A4
#define sensVoltCel3 A3
#define sensCurrent A2

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
bool TimeToStabilize = false;
int MaxonLinks_pwm = 0;
int MaxonRechts_pwm = 0;
int Farnell_pwm = 0;



void setup() {
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  if (serialAllowmend){
    while (! Serial) { delay(1); }
  }

  pinMode(RelaisBlower, OUTPUT);
  pinMode(RelaisSysteem, OUTPUT);
  pinMode(ledBlauw, OUTPUT);
  pinMode(ledRood, OUTPUT);
  pinMode(Maxon_ALinks, OUTPUT);
  pinMode(Maxon_ARechts, OUTPUT);
  pinMode(Maxon_BLinks, OUTPUT);
  pinMode(Maxon_BRechts, OUTPUT);
  pinMode(Farnell_ALinks, OUTPUT);
  pinMode(Farnell_ARechts, OUTPUT);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  Serial.println("Both in reset mode...(pins are low)");
  
  setupTimer();
  setID();
  BlowersOn(); 
  Serial.println("SETUP DONE");
}



void loop() {
 if(stabalize){
   //read_sensors();
    maxonMotorLinks(0, MaxonLinks_pwm);
    maxonMotorRechts(0, MaxonRechts_pwm);
    FarnellMotor(0, Farnell_pwm);
 }
}



void setupTimer() {
  //https://community.robotshop.com/forum/t/arduino-101-timers-and-interrupts/13072
  //http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html

  // TIMER 1 for interrupt frequency 3 Hz:
  cli();       // stop interrupts
  TCCR1A = 0;  // set entire TCCR1A register to 0
  TCCR1B = 0;  // same for TCCR1B
  TCNT1 = 0;   // initialize counter value to 0
  // set compare match register for 3.0000480007680124 Hz increments
  OCR1A = 20832;  // = 16000000 / (256 * 3.0000480007680124) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 256 prescaler
  TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();  // allow interrupts
}

ISR(TIMER1_COMPA_vect) {
  static int count = 0;
  MeassureAmperage();
  
  if(serialAllowmend){
    Serial.print("Counter: ");
    Serial.println(count);
  }

  if(count >= 30){
    TimeToStabilize = true;
  }

  if (count >= 60){
    MeassureBatteryVoltage();
    count = 0;
  }
  count++;
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
    ERROR(9);
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!TOF2_right.begin(VL530x_ADDRESS_right)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    ERROR(9);
    while(1);
  }
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if(!TOF3_side.begin(VL530x_ADDRESS_side)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    ERROR(9);
    while(1);
  }
}



void MeassureAmperage() {
  #define NoLoadOuput 2.5  //Volt
  #define sensitivity 0.66
  const int currentThreshold = 7;
  int adc = analogRead(sensCurrent);
  float voltage = adc * 5 / 1024.0;
  float current = (voltage - NoLoadOuput) / sensitivity * 10;

  if(serialAllowmend)
  {
    Serial.print("Current: ");
    Serial.println(current);
  }

  if (current >= currentThreshold) {
    static int aboveThreshold = 0;
    aboveThreshold++;
    Serial.println("Threshold current above 7A");
    if(aboveThreshold == 2){    //doesn't reset until full reset
      ERROR(1);
    }
  }
}



void MeassureBatteryVoltage() {
  #define VoltageThreshold 3
  int value_1, value_2, value_3 = 0;
  float VoltageCell_1, VoltageCell_2, VoltageCell_3;
  int weerstandSerieCel1 = 3000;  //R1 + R2 + R3
  int weerstandSerieCel2 = 2000;  //R1 +R2
  int weerstandSerieCel3 = 220;   //R3
  int R1, R2 = 1000;
  int R3 = 220;

  value_1 = analogRead(sensVoltCel1);
  VoltageCell_1 = value_1 * (5.0 / 1024) * ((weerstandSerieCel1) / R2);
  value_2 = analogRead(sensVoltCel2);
  VoltageCell_2 = value_2 * (5.0 / 1024) * ((weerstandSerieCel2) / R2);
  value_3 = analogRead(sensVoltCel3);
  VoltageCell_3 = value_3 * (5.0 / 1024) * ((weerstandSerieCel3) / R3); 

  if(serialAllowmend){
    Serial.print("CEL 1:");      Serial.println(VoltageCell_1);
    Serial.print("CEL 2:");      Serial.println(VoltageCell_2);
    Serial.print("CEL 3:");      Serial.println(VoltageCell_3);
  }

  if (VoltageCell_1 <= VoltageThreshold || VoltageCell_2 <= VoltageThreshold || VoltageCell_3 <= VoltageThreshold) {
    ERROR(2);
  }
}



void read_sensors() {
  static int angle;

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

  if((sensor1 && sensor2) < 500){
    angle = angleMeasurement(sensor1, sensor2);
  }
  else{
    angle = 0;
  }
  Serial.print("\tangle: ");
  Serial.println(angle);
}

int angleMeasurement(int afstandLinks, int afstandRechts)
{
  const int sensorsApartFromEachother = 270;   //mm
  int distanceDiff = afstandLinks - afstandRechts;
  float radian = atan2(distanceDiff, sensorsApartFromEachother);
  float angle = radian * 180/3.14159265;
  return angle;
}



void BlowersOn()
{
  digitalWrite(RelaisBlower, HIGH);
  digitalWrite(RelaisSysteem, HIGH);
}

void maxonMotorLinks (int direction, int speed)
{
  const int LINKS = 1;
  const int RECHTS = 2;

  if(direction == LINKS){                 //Rotate left
    analogWrite(Maxon_ARechts, 0);  
    analogWrite(Maxon_ALinks, speed);
  }
 if(direction == RECHTS){                 //rotate right
    analogWrite(Maxon_ALinks, 0);
    analogWrite(Maxon_ARechts, speed);  
  }
   if(direction == 0){                    //stop
    analogWrite(Maxon_ARechts, 0);  
    analogWrite(Maxon_ALinks, 0);
  }
}

void maxonMotorRechts (int direction, int speed)
{
  const int LINKS = 1;
  const int RECHTS = 2;

  if(direction == LINKS){                 //Rotate left
    analogWrite(Maxon_BRechts, 0);  
    analogWrite(Maxon_BLinks, speed);
  }
 if(direction == RECHTS){                 //rotate right
    analogWrite(Maxon_BLinks, 0);
    analogWrite(Maxon_BRechts, speed);  
  }
   if(direction == 0){                    //stop
     analogWrite(Maxon_BRechts, 0);  
    analogWrite(Maxon_BLinks, 0);
  }
}

void FarnellMotor (int direction, int speed)
{
  const int LINKS = 1;
  const int RECHTS = 2;

  if(direction == LINKS){                 //Rotate left
    analogWrite(Farnell_ARechts, 0);  
    analogWrite(Farnell_ALinks, speed);
  }
 if(direction == RECHTS){                 //rotate right
    analogWrite(Farnell_ALinks, 0);
    analogWrite(Farnell_ARechts, speed);  
  }
   if(direction == 0){                    //stop
    analogWrite(Farnell_ALinks, 0);  
    analogWrite(Farnell_ARechts, 0);
  }
}

void shutdown()
{
  Serial.println("SHUTDOWN");
  digitalWrite(RelaisBlower, LOW);
  digitalWrite(RelaisSysteem, LOW);
  maxonMotorLinks(0, MaxonLinks_pwm);
  maxonMotorRechts(0, MaxonRechts_pwm);
  FarnellMotor(0, Farnell_pwm);
}



void ERROR(int errorCode)
{
  switch (errorCode) {
    case 1:
      Serial.println("Error #1");
      Serial.println("Current above threshold");
      //logica toevoegen
      shutdown();
      errorLed(1);
      break;

    case 2:
      Serial.println("Error #2");
      Serial.println("Voltage below threshold");
      //logica toevoegen
      shutdown();
      errorLed(2);
      break;

    default:
      Serial.println("Unknown error");
      digitalWrite(ledRood, HIGH);
      break;
  }
}

void errorLed(int morseCode)
{
  unsigned long time;
  const int oneSecond = 1000;  //in millisecond
  float timeToggleLed;
  bool status = HIGH;
  float blinkTime = oneSecond / morseCode;
  time = millis();

  while(time <= (time+4000)){
    timeToggleLed = time + blinkTime;

    if(time >= timeToggleLed){
      status = !status;
      digitalWrite(ledRood, status);
    }
  }
}