//Motor Set-Up
  int LeftINA = 13; // Controls the direction of spin for the left motor
  int LeftINB = 12;
  int LeftSpeed = 11; //ENA Pin - Controls the speed for the right motor, must be on a PWM(~)  pin

  int RightINC = 8; //Controls the directino of spin for the right motor
  int RightIND = 9;
  int RightSpeed = 10; //ENB Pin - Controls the speed for the right motor, must be on a PWM(~) pin

//Ultrasonic Set-up
  int trig[3] = {5, 6, 7}; // L, F, R \ pins
  int echo[3] = {A0, A1, A2};
  //float lights[3] = {2, 3, 4}; //Pins for Wall Indicator lights, L, F, R, need to test if pinmode ouput works

  int running = 0; 

void setup() {
  // put your setup code here, to run once:
    //Begin Serial connection
  Serial.begin(9600);

  int L = 0; // to help
  int F = 1;
  int R = 2; 

  //Ultrasonic pin mode set-up
  pinMode(trig[L], OUTPUT); 
  pinMode(trig[F], OUTPUT); 
  pinMode(trig[R], OUTPUT); 
  pinMode(echo[L], INPUT_PULLUP); //input_pullup so you can use the analog pins as digital pins
  pinMode(echo[F], INPUT_PULLUP); 
  pinMode(echo[R], INPUT_PULLUP); 

  //pinMode(lights[L], OUTPUT);// have no clue if this actually works for output pins
  //pinMode(lights[F], OUTPUT);
  //pinMode(lights[R], OUTPUT);

  //Set-up Motors
  pinMode(LeftINA, OUTPUT);
  pinMode(LeftINB, OUTPUT);
  pinMode(LeftSpeed, OUTPUT);

  pinMode(RightINC, OUTPUT);
  pinMode(RightIND, OUTPUT);
  pinMode(RightSpeed, OUTPUT);
}

float ultrasonic(int side) {
  // takes in an integer 0, 1, 2 == LEFT, FRONT, RIGHT, returns float distance 
  //Check to see if there is a wall in the side

  //Trigger the sensor to get a reading
  digitalWrite(trig[side], LOW); //Clear the signal
  delayMicroseconds(2);
  digitalWrite(trig[side], HIGH);
  delayMicroseconds(10);
  digitalWrite(trig[side], LOW);

  //Calculate the distance that the sensor is from the wall
  float duration = pulseIn(echo[side], HIGH); //cm/s
  float distance = (duration*.0343)/2; //cm

  return distance;
}


void forward() {
  //Moves the robot forward by 1 cell, returns nothing
  float dist_fwall_1 = ultrasonic(1);
  analogWrite(LeftSpeed, 100);  //Controlling speed (0  = off and 255 = max speed):    
  analogWrite(RightSpeed, 100); 

  if (dist_fwall_1 >= (14.8)){ //if we know that the wall is further away than the closest a wall can be we know we can move forward
    while (abs(dist_fwall_1 - ultrasonic(1)) >= 16.75) {
      //int speed[2] = {center()};//{100, 100}; 
      //analogWrite(LeftSpeed, speed[0]);  //Controlling speed (0  = off and 255 = max speed), centers itself hopefully:    
      //analogWrite(RightSpeed, speed[1]); 

      digitalWrite(LeftINA, LOW); //Write to move forward 
      digitalWrite(LeftINB, HIGH);
      digitalWrite(RightINC, HIGH);
      digitalWrite(RightIND, LOW);
      delayMicroseconds(120);
    }
  }
  digitalWrite(LeftINA, LOW);
  digitalWrite(LeftINB, LOW);
  digitalWrite(RightINC, LOW);
  digitalWrite(RightIND, LOW);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  forward();

}
