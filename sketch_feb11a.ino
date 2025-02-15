//Motor Set-Up
  int LeftINA = 13; // Controls the direction of spin for the left motor
  int LeftINB = 12;
  int LeftSpeed = 11; //ENA Pin - Controls the speed for the right motor, must be on a PWM(~)  pin

  int RightINC = 8; //Controls the directino of spin for the right motor
  int RightIND = 9;
  int RightSpeed = 10; //ENB Pin - Controls the speed for the right motor, must be on a PWM(~) pin
  
void setup() {
  // put your setup code here, to run once:
  pinMode(LeftINA, OUTPUT);
  pinMode(LeftINB, OUTPUT);
  pinMode(RightINC,  OUTPUT);
  pinMode(RightIND, OUTPUT);

  //(Optional)
  pinMode(11,  OUTPUT); 
  pinMode(RightSpeed, OUTPUT);
  //(Optional)
}

void loop() {
  // put your main code here, to run repeatedly:

  //Controlling speed (0  = off and 255 = max speed):     
  //(Optional)
  analogWrite(11, 255); //ENA  pin
  analogWrite(RightSpeed, 255);
  //(Optional)
  
  digitalWrite(LeftINA,  LOW);
  digitalWrite(LeftINB, HIGH);

  digitalWrite(RightINC, HIGH);
  digitalWrite(RightIND, LOW);
  }

