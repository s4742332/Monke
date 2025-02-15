//Ultrasonic Set-up
  float duration[3], distance[3]; // L, R, F duration and distance
  float trig[3] = {5, 6, 7}; // L, R, F \ pins
  float echo[3] = {16, 14, 15};

void setup() {
  // Set-Up Ultrasonics
    pinMode(echo[0], INPUT_PULLUP);
    pinMode(trig[0], OUTPUT);
    pinMode(echo[1], INPUT_PULLUP);
    pinMode(trig[1], OUTPUT);
    pinMode(echo[2], INPUT_PULLUP);
    pinMode(trig[2], OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(trig[0], LOW);
  delayMicroseconds(2);
  digitalWrite(trig[0], HIGH);
  delayMicroseconds(10);
  digitalWrite(trig[0], LOW);

  duration[0] = pulseIn(echo[0], HIGH); distance[0] = (duration[0]*.0343)/2;
  Serial.println("Left: ");  Serial.println(distance[0]); Serial.println(duration[0]);

  //digitalWrite(trig[1], LOW);
  //delayMicroseconds(2);
  //digitalWrite(trig[1], HIGH);
  //delayMicroseconds(10);
  //digitalWrite(trig[1], LOW);

  //duration[1] = pulseIn(echo[1], HIGH); distance[1] = (duration[1]*.0343)/2;
  //Serial.println("Right: ");  Serial.println(distance[1]); Serial.println(duration[1]);

  //digitalWrite(trig[2], LOW);
  //delayMicroseconds(2);
  //digitalWrite(trig[2], HIGH);
  //delayMicroseconds(10);
  //digitalWrite(trig[2], LOW);

  //duration[2] = pulseIn(echo[2], HIGH); distance[2] = (duration[2]*.0343)/2;
  //Serial.println("Front: ");  Serial.println(distance[2]); Serial.println(duration[2]);


}
