#define start_button 2
#define emergency_button 3  //external interrupt
#define motorTangential 10
#define stuurMotorRechts 11
#define stuurMotorLinks 12
#define blowers 13

volatile bool eState = false;
bool lastButtonState = LOW;
bool blowerState = LOW;

void myISR()
{
  Serial.println("trigger");
  eState = !eState;
}

void init_UAV()
{
  
}



void setup() 
{
  Serial.begin(115200);
  
  pinMode(start_button, INPUT_PULLUP);
  pinMode(emergency_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(emergency_button), myISR, FALLING);
  
  pinMode(blowers, OUTPUT);
  pinMode(stuurMotorLinks, OUTPUT);
  pinMode(stuurMotorRechts, OUTPUT);
}



void loop() 
{

  init_UAV();

  if(eState == false){
    byte buttonState = digitalRead(start_button);
    if (buttonState != lastButtonState) {
      lastButtonState = buttonState;
      if (buttonState == LOW) {
        blowerState = (blowerState == HIGH) ? LOW: HIGH;
        digitalWrite(blowers, blowerState);
      }
    }
  }
  if(eState == true){
    digitalWrite(blowers, LOW);
  }
}
