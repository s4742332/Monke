//Set-Up Indicators
  float Lights[3] = {3, 4, 5}; //Pins for Lights, L, R, F
  
  //int Walld = 5; //distance (cm) in which a Wall is definitely in front/surrounding the mouse
  //float Walls[3]; //L, R, F in that order, boolean

void setup() {
  //Set-up pinmode 
  for (byte i = 0; i=2; i++) {
    pinMode(Lights[i], OUTPUT); //set L, R, F indicators
  }
}

void loop() {
  // Test if they work
  for (byte i = 0; i =2; i++) { // for each ultrasonic: L, R, F
      analogWrite(Lights[i], HIGH); // Associated indicator light 
      delay(30);
      analogWrite(Lights[i], LOW);
      delayMicroseconds(10);
    }
}
