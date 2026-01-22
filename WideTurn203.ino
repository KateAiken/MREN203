
int EA = 3;  // Black Wheel PWM pin (must be a PWM pin)
int I1 = 4;  //Light Grey                                                                                                                                                                                                                                           // Wheel direction digital pin 1
int I2 = 2;  // White
int EB = 5;  //Orange
int I3 = 7;  //Brown
int I4 = 8;  //Red
int time = 0;
int max_time = 10;  // Wheel direction digital pin 2

void setup() {
  // Configure digital pins for output
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
  stopmotor();
}

void loop() {

  delay(100);
  // Play with this code to write open loop commands to a wheel motor
  turnrightwide(24);
  turnleftwide(23);
 
  // moveforward(1);


  // turnright();

  // movebackwards(1);

  // turnleft();

  // movebackwards(1);

  // turnright();

  // moveforward(1);

  // turnleft();


  //stopmotor();
}

void movebackwards(int time) {
  int u;
  int start = 0;
  while (start < time) {
    for (u = 100; u <= 255; u += 5) {
      // Select a direction
      digitalWrite(I1, LOW);
      digitalWrite(I2, HIGH);
      digitalWrite(I3, HIGH);
      digitalWrite(I4, LOW);
      // PWM command to the motor driver
      analogWrite(EA, u);
      analogWrite(EB, u);
      // Brief delay (perhaps not necessary)
      delay(100);
    }
    start++;
  }
  stopmotor();
}

void moveforward() {

  for (int u = 100; u <= 255; u += 25) {
    // Select a direction
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    // PWM command to the motor driver
    analogWrite(EA, u);
    analogWrite(EB, u);
    // Brief delay (perhaps not necessary)
    delay(100);
  }

  //stopmotor();
}

void turnright() {
  for (int v = 100; v <= 255; v += 8) {
    // Select a direction
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    // PWM command to the motor driver
    analogWrite(EA, v);
    analogWrite(EB, v);
    // Brief delay (perhaps not necessary)
    delay(100);
  }
  //delay(560);
  //delay is 560 for full turn
  stopmotor();
}

void turnleft() {
  for (int v = 100; v <= 255; v += 20) {
    // Select a direction
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    // PWM command to the motor driver
    analogWrite(EA, v);
    analogWrite(EB, v);
    // Brief delay (perhaps not necessary)
    delay(100);
  }
  //delay(580);
  //delay=580 for full turn
  //stopmotor();
}

void stopmotor() {
  digitalWrite(I1, HIGH);
  digitalWrite(I2, HIGH);
  digitalWrite(I3, HIGH);
  digitalWrite(I4, HIGH);
  delay(100);
}

void turnrightwide(int turn_r) {
  for (int i = 0; i < turn_r; i++) {
    //FORWARD
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);

    analogWrite(EA, 150);
    analogWrite(EB, 230);
    delay(200);
    //LEFT
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);

    analogWrite(EA, 150);
    analogWrite(EB, 230);
    delay(200);
  }
}

void turnleftwide(int turn_r) {

  for (int i = 0; i < turn_r; i++) {
    //FORWARD
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);

    analogWrite(EA, 230);
    analogWrite(EB, 150);
    delay(200);
    //LEFT
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);

    analogWrite(EA, 230);
    analogWrite(EB, 150);
    delay(200);
  }
}