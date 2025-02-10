// Libraries
  #include <Stepper.h>

//Set-up Motors
  const int STEPS = 2048; // Full revolution for the 28BYJ-48 motor in full-step mode?
  const int MaxSpeed = 255; 
  const int Stop = 0;
  Stepper Leftmotor(STEPS, 10, 11, 12, 13); 
  Stepper Rightmotor(STEPS, 8, 9, 6, 7); //9, 6, 7, 8

void setup() {
  // Set the motor speed (RPM)
  Leftmotor.setSpeed(10);  // Set a lower speed to reduce RPM (adjust to your needs)
  Rightmotor.setSpeed(10);  // Set a lower speed to reduce RPM (adjust to your needs)
  Serial.begin(9600);
}

void loop() {
  for (int s =0; s < STEPS; s+2) {
    Leftmotor.step(s);
    Rightmotor.step(s);
    Serial.println("Stepped");
  }
}
