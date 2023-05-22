#define Maxon_ALinks 2
#define Maxon_ARechts 3
#define Maxon_BLinks 4
#define Maxon_BRechts 5
#define Farnell_ALinks 6
#define Farnell_ARechts 7

int MaxonLinks_pwm = 0;
int MaxonRechts_pwm = 0;
int Farnell_pwm = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode(Maxon_ALinks, OUTPUT);
  pinMode(Maxon_ARechts, OUTPUT);
  pinMode(Maxon_BLinks, OUTPUT);
  pinMode(Maxon_BRechts, OUTPUT);
  pinMode(Farnell_ALinks, OUTPUT);
  pinMode(Farnell_ARechts, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  maxonMotorLinks(0, MaxonLinks_pwm);
  maxonMotorRechts(0, MaxonRechts_pwm);
  FarnellMotor(0, Farnell_pwm);

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