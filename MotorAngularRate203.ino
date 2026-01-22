/**

NEW SKETCH
 * @file motor-angular-rate.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to estimate motor speed from encoder.
 * @version 2.1
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2021-2022
 *
 */

int EA = 3;  // Black Wheel PWM pin (must be a PWM pin)
int I1 = 4;  //Light Grey                                                                                                                                                                                                                                           // Wheel direction digital pin 1
int I2 = 2;  // White
int EB = 5;  //Orange
int I3 = 7;  //Brown
int I4 = 8;  //Red

// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_L_A = 9;
const byte SIGNAL_L_B = 10;
const byte SIGNAL_R_A = 11;
const byte SIGNAL_R_B = 12;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_R = 0.0;
double omega_L = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks() {
  if (digitalRead(SIGNAL_L_B) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_L++;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_L--;
  }
  if (digitalRead(SIGNAL_R_A) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_R--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_R++;
  }
}

void setup() {
  // Open the serial port at 9600 bps
  Serial.begin(9600);

  // Set the pin modes for the motor driver
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  // Set the pin modes for the encoders
  pinMode(SIGNAL_L_A, INPUT);
  pinMode(SIGNAL_L_B, INPUT);
  pinMode(SIGNAL_R_A, INPUT);
  pinMode(SIGNAL_R_B, INPUT);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_L_A), decodeEncoderTicks, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_R_A), decodeEncoderTicks, RISING);

  // Print a message
  Serial.print("Program initialized.");
  Serial.print("\n");
}

void loop() {
  // Get the elapsed time [ms]
  t_now = millis();

  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)encoder_ticks_L / (double)TPR) * 1000.0 / (double)(t_now - t_last);
    omega_R = 2.0 * PI * ((double)encoder_ticks_R / (double)TPR) * 1000.0 / (double)(t_now - t_last);

    // Print some stuff to the serial monitor
    Serial.print("LEFT ticks: ");
    Serial.print(encoder_ticks_L);
    Serial.print("\t");
    Serial.print("LEFT speed: ");
    Serial.print(omega_L);
    Serial.print(" rad/s\n");

    Serial.print("RIGHT ticks: ");
    Serial.print(encoder_ticks_R);
    Serial.print("\t");
    Serial.print("RIGHT speed: ");
    Serial.print(omega_R);
    Serial.print(" rad/s\n");


    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_L = 0;
    encoder_ticks_R = 0;
  }

  // Set the wheel motor PWM command [0-255]
  u = 128;

  // Select a direction
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);

  // PWM command to the motor driver
  analogWrite(EA, u);
  analogWrite(EB, u);
}
