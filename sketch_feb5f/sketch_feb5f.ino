#include <Arduino_LSM6DS3.h>
int EA = 3;  // Black Wheel PWM pin (must be a PWM pin)
int I1 = 4;  //Light Grey                                                                                                                                                                                                                                           // Wheel direction digital pin 1
int I2 = 2;  // White
int EB = 5;  //Orange
int I3 = 7;  //Brown
int I4 = 8;  //Red
int time = 0;
int max_time = 10;  // Wheel direction digital pin 2

// Variables to store angular rates from the gyro [degrees/s]
float omega_x, omega_y, omega_z;

// Variables to store accelerations [g's]
float a_x, a_y, a_z;

// Variables to store sample rates from sensor [Hz]
float a_f, g_f;

byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_L_A = 9;
const byte SIGNAL_L_B = 10;
const byte SIGNAL_R_A = 11;
const byte SIGNAL_R_B = 12;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;


// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_R = 0.0;
double omega_L = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 50;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

//Wheel radius
double r = 0.0625;

// Estimated Speed
double v_L = 0;
double v_R = 0;

// Things
double v = 0;
double omega = 0;

// constant
const double ELL = 0.2775;

// e's
double e_L;
double e_R;

// u's
int u_L = 200;
int u_R = 200;

//e_sum's
double e_sum_L = 0;
double e_sum_R = 0;

const double I_MAX = 40.0;
double kFF_L = 220;  // feed-forward gain (tune once)
double kFF_R = 210;
const int PWM_MIN = 50;  //motor dead-zone clamp
const int PWM_MAX = -100;  //motor dead-zone clamp

double v_des_L = 0;
double v_des_R = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeLeftEncoder() {
  if (digitalRead(SIGNAL_L_B) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_L--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_L++;
  }
}
void decodeRightEncoder() {
  if (digitalRead(SIGNAL_R_B) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_R++;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_R--;
  }
}

void checkIMU() {
  // Read from the accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(a_x, a_y, a_z);

    /// Print the accelerometer measurements to the Serial Monitor
    Serial.print(a_x);
    Serial.print("\t");
    Serial.print(a_y);
    Serial.print("\t");
    Serial.print(a_z);
    Serial.print(" g\t\t");
  }

  // Read from the gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(omega_x, omega_y, omega_z);
    // average(omega_x, omega_y, omega_z);


    // Print the gyroscope measurements to the Serial Monitor
    Serial.print(omega_x);
    Serial.print("\t");
    Serial.print(omega_y);
    Serial.print("\t");
    Serial.print(omega_z);
    Serial.print(" deg/s\n");
  }
}

void checkEncoder() {
  // Get the elapsed time [ms]
  t_now = millis();

  if (t_now - t_last >= T) {

    double dt = (t_now - t_last) / 1000.0;

    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)encoder_ticks_L / (double)TPR) / dt;
    omega_R = 2.0 * PI * ((double)encoder_ticks_R / (double)TPR) / dt;
    v_L = r * omega_L;
    v_R = r * omega_R;

    v = 0.5 * (v_L + v_R);      // [m/s]
    omega = (v_R - v_L) / ELL;  // [rad/s]

    if (abs(v) < 0.01) {  //Reset integral when stopped
      e_sum_L = 0;
      e_sum_R = 0;
    }

    // Print some stuff to the serial monitor
    // Serial.print("LEFT ticks: ");
    // Serial.print(encoder_ticks_L);
    // Serial.print("\t");
    Serial.print("LEFT speed: ");
    Serial.print(omega_L);
    Serial.print(" rad/s\n");

    // Serial.print("RIGHT ticks: ");
    // Serial.print(encoder_ticks_R);
    // Serial.print("\t");
    Serial.print("RIGHT speed: ");
    Serial.print(omega_R);
    Serial.print(" rad/s\n");

    Serial.print("Velocity: ");
    Serial.print(v);
    Serial.print(" Turn Rate: ");
    Serial.print(omega);
    Serial.print("\n");

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_L = 0;
    encoder_ticks_R = 0;
  }
}

// Wheel speed PI controller function
int PI_controller(double e_now, double e_int, double k_P, double k_I) {
  int u;
  u = (int)((k_P * e_now) + (k_I * e_int));
  if (u > 255) {
    u = 255;
  } else if (u < -255) {
    u = -255;
  }
  return u;
}

void calcError(double speed, double turnRate) {
  v_des_L = speed - (turnRate * (ELL / 2));  // Speed doesnt need to be rad/s code changes turn rate to m/s. units work
  Serial.print("Desired Speed Left: ");
  Serial.print(v_des_L);
  e_L = (v_des_L - v_L);

  v_des_R = speed + (turnRate * (ELL / 2));
  Serial.print("    Desired Speed Right: ");
  Serial.print(v_des_R);
  Serial.print("\n");
  e_R = (v_des_R - v_R);
  Serial.print("E Left: ");
  Serial.print(e_L);
  Serial.print("    E Right: ");
  Serial.print(e_R);
  Serial.print("\n");

  e_sum_L += e_L;
  e_sum_R += e_R;
    //e_sum_L = constrain(e_sum_L, -I_MAX, I_MAX);
    //e_sum_R = constrain(e_sum_R, -I_MAX, I_MAX);
}

void setup() {
  // Configure digital pins for output
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
  stopmotor();

  // Open the serial port at 115200 bps
  Serial.begin(9600);

  // Wait for serial connection before starting
  while (!Serial) {
    delay(10);
  }

  Serial.println();

  // Check that the board is initialized
  if (!IMU.begin()) {
    // Print an error message if the IMU is not ready
    Serial.print("Failed to initialize IMU :(");
    Serial.print("\n");
    while (1) {
      delay(10);
    }
  }

  // Read the sample rate of the accelerometer and gyroscope
  a_f = IMU.accelerationSampleRate();
  g_f = IMU.gyroscopeSampleRate();

  // Print these values to the serial window
  Serial.print("Accelerometer sample rate: ");
  Serial.println(a_f);
  Serial.print("Gyroscope sample rate: ");
  Serial.println(g_f);

  // Set the pin modes for the encoders
  pinMode(SIGNAL_L_A, INPUT);
  pinMode(SIGNAL_L_B, INPUT);
  pinMode(SIGNAL_R_A, INPUT);
  pinMode(SIGNAL_R_B, INPUT);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_L_A), decodeLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_R_A), decodeRightEncoder, RISING);

  // Print a message
  Serial.print("Program initialized.");
  Serial.print("\n");
}

void loop() {

  fastforward();
  // turnright();
  checkEncoder();
  //checkIMU();
  calcError(0, -3);


  int u_ff_L = kFF_L * v_des_L;  //feed forward
  int u_ff_R = kFF_R * v_des_R;

  int u_pi_L = PI_controller(e_L, e_sum_L, 200, 100);  //PI correction
  int u_pi_R = PI_controller(e_R, e_sum_R, 200, 100);


  u_L = constrain(u_ff_L + u_pi_L, -255, 255);  //Combination of feed forward and PI
  u_R = constrain(u_ff_R + u_pi_R, -255, 255);

  if (u_L > 0 && u_L < PWM_MIN) {  //Prevents motor-dead zone
    u_L = PWM_MIN;
  }
   else if (u_L < 0 && u_L > PWM_MAX) {  //Prevents motor-dead zone
     u_L = PWM_MAX;
   }
  if (u_R > 0 && u_R < PWM_MIN) {
    u_R = PWM_MIN;
  }
   else if (u_R < 0 && u_R > PWM_MAX) {
     u_R = PWM_MAX;
   }

  //   u_L = PI_controller(e_L, e_sum_L, 170, 18);
  Serial.print("PWM Left: ");
  Serial.print(u_L);
  //   u_R = PI_controller(e_R, e_sum_R, 170, 18);
  Serial.print("    PWM Right: ");
  Serial.print(u_R);
  Serial.print("\n");
}


void fastforward() {
  if (u_L < 0 && u_R > 0) {
    u_L *= -1;
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    // PWM command to the motor driver
    analogWrite(EA, u_R);
    analogWrite(EB, u_L);
  } else if (u_R < 0 && u_L > 0) {
    u_R *= -1;
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    // PWM command to the motor driver
    analogWrite(EA, u_R);
    analogWrite(EB, u_L);
  } else {
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    // PWM command to the motor driver
    analogWrite(EA, u_R);
    analogWrite(EB, u_L);
    //delay(500);
  }
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

  // Select a direction
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);
  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);
  // PWM command to the motor driver
  analogWrite(EA, u_L);
  analogWrite(EB, u_R);
  // Brief delay (perhaps not necessary)
  // delay(100);

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
    checkIMU();
    checkEncoder();
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
    checkIMU();
    checkEncoder();
    delay(200);
  }
}