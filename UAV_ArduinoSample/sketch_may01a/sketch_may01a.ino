#define blowers 13
#define start_button 3

byte lastButtonState = LOW;
byte blowerState = LOW;

void setup() {
  pinMode(blowers, OUTPUT);
  pinMode(start_button, INPUT);
}

void loop() {
  byte buttonState = digitalRead(start_button);
  if (buttonState != lastButtonState) {
    lastButtonState = buttonState;
    if (buttonState == LOW) {
      blowerState = (blowerState == HIGH) ? LOW: HIGH;
      digitalWrite(blowers, blowerState);
    }
  }
}
