//Ultrasonic Set-up
  const int trigPin = 9; // test
  const float echo[3] = {10, 11, 12}; // L, R, F echo pins
  float duration[3], distance[3]; // L, R, F duration and distance

//Indicators Set-up
  const float Lights[3] = {3, 4, 5}; //Pins for Lights, L, R, F

//Walls
  const int Walld = 5; //distance (cm) in which a Wall is definitely in front/surrounding the mouse
  float Walls[3]; //L, R, F in that order, boolean inside

void setup() {
//Set-up pinmodes
  for (byte i = 0; i=2; i++) {
    pinMode(Lights[i], OUTPUT); //set L, R, F indicators
    pinMode(echo[i], INPUT); //set L, R, F ultrasonic echopins
  }
  pinMode(trigPin, OUTPUT); // Trigger pin set-up
  Serial.begin(9600); //begin terminal
}


void loop() {
//Check to see if there is a wall

  //Trigger the senors to get a reading
  digitalWrite(trigPin, LOW); //Clear the signal
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Durations
  duration[0] = pulseIn(echo[0], HIGH);
  duration[1] = pulseIn(echo[1], HIGH);
  duration[2] = pulseIn(echo[2], HIGH);

  //Calc distance
  distance[0] = (duration[0]*.0343)/2;
  distance[1] = (duration[1]*.0343)/2;
  distance[2] = (duration[2]*.0343)/2;

  //Print Results to terminal
  Serial.println("Left: ");  Serial.print(distance[0]);
  Serial.println("Right: "); Serial.print(distance[1]);
  Serial.println("Front: "); Serial.print(distance[2]);

  //Indicate if Wall
  for (byte i = 0; i =3; i++) { // for each ultrasonic: L, R, F
    if (distance[i] <= Walld) { // check if the the distance is close to us or not
      analogWrite(Lights[i], HIGH); // Associated indicator light 
      Walls[i] = true; //Wall is there
      Serial.println(Walls[i]);
    }
    else {
      analogWrite(Lights[i], LOW);
      Walls[i] = false; // No Wall
      Serial.println(Walls[i]);
    }
  } 

}
