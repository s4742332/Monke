//Ultrasonic Set-up
  int trig[3] = {5, 6, 7}; // L, F, R \ pins
  int echo[3] = {16, 14, 15};
  float lights[3] = {2, 3, 4}; //Pins for Wall Indicator lights, L, F, R, need to test if pinmode ouput works

//Motor Set-Up
  int LeftINA = 13; // Controls the direction of spin for the left motor
  int LeftINB = 12;
  int LeftSpeed = 11; //ENA Pin - Controls the speed for the right motor, must be on a PWM(~)  pin

  int RightINC = 8; //Controls the directino of spin for the right motor
  int RightIND = 9;
  int RightSpeed = 10; //ENB Pin - Controls the speed for the right motor, must be on a PWM(~) pin
  
//Global variables
  const int Walld = 4;//cm
  const int cell_wall = 16.8; //cm
  int bot_orientation=0; //0 is F, works in a circle (90 is LR and so on)
  int bot_position[2]={0,0}; // co-ordinate position of the bot (x, y) related to the coordinates

void setup() {
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

  pinMode(lights[L], OUTPUT);// have no clue if this actually works for output pins
  pinMode(lights[F], OUTPUT);
  pinMode(lights[R], OUTPUT);

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

bool walls() {
  //Checks the surrounding walls in a cell, returns True or False in order F R B L array
  bool iswalls[4];

  for (int i = 0; i =2; i++) { // for each ultrasonic in order of // L, F, R, B, reordered to match the standard order
    if (ultrasonic(i) <= Walld) { // check if the the distance is close to robot or not
      analogWrite(lights[i], HIGH); // Associated indicator light turns on if a wall is dedicated
      iswalls[i-1] = true; //Wall is there, reordered to match overall format
    }
    else {
      analogWrite(lights[i], LOW);
      iswalls[i-1] = false; //Wall is not there
    }
    iswalls[2] = false; //No wall at the back always set to false 
  }
  return iswalls; //iswalls comes out in order F, R, B, L, 
}

int center() {
  //updates the speed that the motors should move
  float dist_lwall = ultrasonic(0);
  float dist_rwall = ultrasonic(2);
  int speed[2] = {100, 100}; // L R
  int L = 0;
  int R = 1;

  if (abs(dist_lwall - dist_rwall) < 0.5) { // Minimal Difference between the walls means we are centered
    speed[0] = speed[1] = 100;
    return speed;}// Standard Speed
  else if ((dist_lwall - dist_rwall) < -3.8 ) {//Closest to the Left Wall, Maximum Difference (this means that the wall might be further away than average)
    //left motor much faster than right motor
    speed[L] += 10;
    return speed;} 
  else if ( -3.8 < (dist_lwall - dist_rwall) < -0.5 ) { //Closest to the Left Wall, Average Difference (this means that the walls are average)
    //left motor just  faster than right motor
    speed[L] += 5;
    return speed;}
  else if ((dist_lwall - dist_rwall) > 3.8 ) {//Closest to the Right Wall, Maximum Difference (this means that the wall might be further away than average)
    //right motor much faster than left motor
    speed[R] += 10;
    return speed;}
  else if ( 0.5 < (dist_lwall - dist_rwall) < 3.8 ) { //Closest to the Right Wall, Average Difference (this means that the walls are average)
    //right motor just  faster than left motor
    speed[R] += 5;
    return speed;}
}

void update_position(int movement) { // L, F, R == 0 , 1 , 2
  //updates the current cell position of the robot based upon movements given to the robot
  int right_turn = 90; // NOTE: orientation: front = 0 or +360, right = +90, back = +180, left = +270 
  int left_turn = -90;
  int x = 0; //index of the bot position for clarity
  int y = 1;

  // Pre-correct the orientation of the bot 
  if (bot_orientation <0) {bot_orientation = 360;}
  else if (bot_orientation > 360) {bot_orientation = -360;}  // Make a line to say if direction <0 then +360 and if direction >360 then -360 to make sure it is 0<direction<360

  //positioning moment
  if (movement == 1 && ((bot_orientation == 0 || bot_orientation == 360) || bot_orientation == -360)) { 
    bot_position[y] += 1; // move forward and is facing forward =? (0, 1)
  }
  else if (movement == 1 && (bot_orientation == 270||bot_orientation == -270)) {
    bot_position[y] -= 1; // move forward and is facing backward =? (0, -1)
  }
  else if (movement == 1 && (bot_orientation == 90)) {
    bot_position[x] += 1; // move forward and is facing right =? (1, 0)
  }
  else if (movement == 1 && (bot_orientation == 90)) { 
    bot_position[x] += 1; // move forward and is facing right =? (-1, 0)
  }
  else if (movement = 2) { //right
    bot_orientation += right_turn; // re-orient
  }
  else if (movement =0) { //left
    bot_orientation += left_turn; // re-orient
  }
  //Post correct orientation
  if (bot_orientation <0) {bot_orientation = 360;}
  else if (bot_orientation > 360) {bot_orientation = -360;}
}

void forward() {
  //Moves the robot forward by 1 cell, returns nothing
  //Call front ultrasonic sensor grab a temporary distance to the front wall
  float dist_fwall_1 = ultrasonic(1);
  analogWrite(LeftSpeed, 100);  //Controlling speed (0  = off and 255 = max speed):    
  analogWrite(RightSpeed, 100); 

  if (dist_fwall_1 >= (cell_wall-2)){ //if we know that the wall is further away than the closest a wall can be we know we can move forward
    while (abs(dist_fwall_1 - ultrasonic(1)) >= 16.75) {
      int speed[2] = {center()};//{100, 100}; 
      analogWrite(LeftSpeed, speed[0]);  //Controlling speed (0  = off and 255 = max speed), centers itself hopefully:    
      analogWrite(RightSpeed, speed[1]); 

      digitalWrite(LeftINA, HIGH); //Write to move forward 
      digitalWrite(LeftINB, LOW);
      digitalWrite(RightINC, HIGH);
      digitalWrite(RightIND, LOW);
      delay(2);
    }
  }
  digitalWrite(LeftINA, LOW);
  digitalWrite(LeftINB, LOW);
  digitalWrite(RightINC, LOW);
  digitalWrite(RightIND, LOW);
  update_position(1);
  
}

void rotate(int direction) {
  // rotates 90 degrees either left or right (0, 2) sticking with the previous formats
  analogWrite(LeftSpeed, 100);  //Controlling speed (0  = off and 255 = max speed):    
  analogWrite(RightSpeed, 100); 
  float distlwall = ultrasonic(0); //left wall distance
  float distfwall = ultrasonic(1); //front wall distance
  float distrwall = ultrasonic(2); //right wall distance

  switch (direction) {
    case (0) : // Turn Left
       // Wait until the left and right wall distances stabilize
      while (abs(ultrasonic(0) - distlwall) > 1 || abs(ultrasonic(2) - distrwall) > 1) {
        delay(10); // Allow sensors to update
        digitalWrite(LeftINA, HIGH);
        digitalWrite(LeftINB, LOW);
         //maybe right motors here too
        delay(10);
        }
      
      digitalWrite(LeftINA, LOW); //Turn motors off after turn
      digitalWrite(LeftINB, LOW);

      update_position(0);
      break;
    
    case (2) : // Turn Right
             // Wait until the left and right wall distances stabilize
      while (abs(ultrasonic(0) - distlwall) > 1 || abs(ultrasonic(2) - distrwall) > 1) {
        delay(10); // Allow sensors to update
        digitalWrite(RightINC, HIGH);
        digitalWrite(RightIND, LOW);
         //maybe right motors here too
        delay(10);
        }
      
      digitalWrite(RightINC, LOW); //Turn motors off after turn
      digitalWrite(RightIND, LOW);

      update_position(2);
      break;
  }
}




void loop() {
  // put your main code here, to run repeatedly:

}
