int LeftINA = 13; // Controls the direction of spin for the left motor
int LeftINB = 12;
int LeftSpeed = 11; //ENA Pin - Controls the speed for the right motor, must be on a PWM(~)  pin

int RightINC = 8; //Controls the directino of spin for the right motor
int RightIND = 9;
int RightSpeed = 10; //ENB Pin - Controls the speed for the right motor, must be on a PWM(~) pin

int LTrig = 5;
int FTrig = 6;
int RTrig = 7;

int L_Echo = A0;
int F_Echo = A1;
int R_Echo = A2;

int L_LED = 4;
int F_LED = 3;
int R_LED = 2;

int run = 0;
void setup() {
  // put your setup code here, to run once:
  //Set-up Motors
  pinMode(LeftINA, OUTPUT);
  pinMode(LeftINB, OUTPUT);
  pinMode(LeftSpeed, OUTPUT);

  pinMode(RightINC, OUTPUT);
  pinMode(RightIND, OUTPUT);
  pinMode(RightSpeed, OUTPUT);

  //Echo input pins
  pinMode(L_Echo, INPUT);
  pinMode(F_Echo, INPUT);
  pinMode(R_Echo, INPUT);

  //Trig output ints
  pinMode(LTrig, OUTPUT);
  pinMode(FTrig, OUTPUT);
  pinMode(RTrig, OUTPUT);

  //Indicator output pins
  pinMode(L_LED, OUTPUT);
  pinMode(F_LED, OUTPUT);
  pinMode(R_LED, OUTPUT);

  Serial.begin(9600);

  Serial.println("Set-up Complete");
}

void indicate (int direction) {
  switch(direction) {
    case 0:
      digitalWrite(L_LED, HIGH);
      break;
    case 1:
      digitalWrite(F_LED, HIGH);
      break;
    case 2:
      digitalWrite(R_LED, HIGH);
      break;
    default:
      Serial.println("Invalid direction received");
      break;
  }
}
float emit_pulse(int side) {
  // takes in an integer 0, 1, 2 == LEFT, FRONT, RIGHT, returns float distance 
  //Check to see if there is a wall in the side
  //Trigger the sensor to get a reading
  float duration;
  float distance;
  switch (side)
  {
    case 0:
      digitalWrite(LTrig, LOW); //Clear the signal
      delayMicroseconds(2);
      digitalWrite(LTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(LTrig, LOW);

      duration = pulseIn(L_Echo, HIGH); //cm/s
      distance = (duration*.0343)/2; //cm
      return distance;
      break;
    case 1:
      digitalWrite(FTrig, LOW); //Clear the signal
      delayMicroseconds(2);
      digitalWrite(FTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(FTrig, LOW);

      duration = pulseIn(F_Echo, HIGH); //cm/s
      distance = (duration*.0343)/2; //cm
      return distance;
      break;
    case 2:
      digitalWrite(RTrig, LOW); //Clear the signal
      delayMicroseconds(2);
      digitalWrite(RTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(RTrig, LOW);

      duration = pulseIn(R_Echo, HIGH,30000); //cm/s
      distance = (duration*.0343)/2; //cm
      return distance;
      break;
    default:
      Serial.println("Invalid side");
      break;
    return distance;
  }
}
void forward() {
  float startDist = 0;
  while (startDist == 0){
    startDist = emit_pulse(1);  // Initial distance to front wall
  }
  float currentDist = startDist;    // Variable to track current distance

  Serial.print("Initial Front Distance: ");
  Serial.println(startDist);

  // Start moving forward
  analogWrite(LeftSpeed, 150);
  analogWrite(RightSpeed, 135);

  digitalWrite(LeftINA, LOW);
  digitalWrite(LeftINB, HIGH);
  digitalWrite(RightINC, HIGH);
  digitalWrite(RightIND, LOW);
  delay(50);
  // Move until the robot has traveled 18 cm
  while ((abs(startDist - currentDist) < 16) or currentDist == 0) { 
    currentDist = emit_pulse(1);  // Continuously update current distance
    Serial.print("Current Distance: ");
    Serial.println(currentDist);
    delay(10);  // Small delay to avoid excessive sensor readings
  }

  // Stop the robot
  analogWrite(LeftSpeed, 100);
  analogWrite(RightSpeed, 100);

  digitalWrite(LeftINA, LOW);
  digitalWrite(LeftINB, LOW);
  digitalWrite(RightINC, LOW);
  digitalWrite(RightIND, LOW);
  delay(500);  // Pause before the next action

  return;
}

void loop() {
  // put your main code here, to run repeatedly:
  //forward();
  if (run == 0){
    forward();
    Serial.println(run);
    run ++;
  }
  

}
