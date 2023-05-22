int serialAllowmend = 1;

void setup() {
  Serial.begin(115200);
  setupTimer();
}



void loop() {
  MeassureBatteryVoltage();
  MeassureAmperage();
  delay(500);
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
  int currentThreshold = 7;
  int trigger = MeassureAmperage();

  if (trigger >= currentThreshold) {
    ERROR(1);
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

  value_1 = analogRead(A0);
  VoltageCell_1 = value_1 * (5.0 / 1024) * ((weerstandSerieCel1) / R2);
  value_2 = analogRead(A1);
  VoltageCell_2 = value_2 * (5.0 / 1024) * ((weerstandSerieCel2) / R2);
  value_3 = analogRead(A2);
  VoltageCell_3 = value_3 * (5.0 / 1024) * ((weerstandSerieCel3) / R3); 

  if(serialAllowmend){
    // char buffer[40];
    // sprintf(buffer, "Baterij voltage - Cel1: %dV \t Cel2: %dV \t Cel3: %dV \t", VoltageCell_1, VoltageCell_2, VoltageCell_3);
    // Serial.println(buffer);
    
    Serial.println("\n\nCEL 1:");
    Serial.println(VoltageCell_1);
    Serial.println("CEL 2:");
    Serial.println(VoltageCell_2);
    Serial.println("CEL 3:");
    Serial.println(VoltageCell_3);
  }

  if (VoltageCell_1 <= VoltageThreshold || VoltageCell_2 <= VoltageThreshold || VoltageCell_3 <= VoltageThreshold) {
    ERROR(2);
  }
}


int MeassureAmperage() {
  #define NoLoadOuput 2.5  //Volt
  #define sensitivity 0.66
  int adc = analogRead(A3);
  float voltage = adc * 5 / 1024.0;
  float current = (voltage - NoLoadOuput) / sensitivity * 10;

  if(serialAllowmend)
  {
    Serial.println("Current: ");
    Serial.println(current);
  }
  return current;
}


void ERROR(int errorCode)
{
  switch (errorCode) {
    case 1:
      Serial.println("Error #1");
      Serial.println("Current above threshold");
      //logica toevoegen
      break;

    case 2:
      Serial.println("Error #2");
      Serial.println("Voltage below threshold");
      //logica toevoegen
      break;

    default:
      Serial.println("Unknown error");
      break;
  }
}

