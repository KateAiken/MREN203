/**
 * @file sharp-range.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca), Thomas Sears (thomas.sears@queensu.ca)
 * @brief Arduino program to read proximity data from a Sharp GP2Y0A21YK.
 * @version 2.1
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */

// Arduino analog input pin to which the Sharp sensor is connected
const byte SHARP_FRONT = A0;
const byte SHARP_LEFT = A1;
const byte SHARP_RIGHT = A2;

// Variables to store the proximity measurement
int sharp_left = 0; // integer read from analog pin
int sharp_right = 0; 
int sharp_front = 0; 
float sharp_range; // range measurement [cm]

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);
}

void loop()
{
    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_left = analogRead(SHARP_LEFT);
    sharp_right = analogRead(SHARP_RIGHT);
    sharp_front = analogRead(SHARP_FRONT);
    int sharp_dist_front = 6950.1* pow(sharp_front, -1.034);
    int sharp_dist_right = 9501.2 *pow(sharp_right, -1.088);
    int sharp_dist_left = 10961 *pow(sharp_left, -1.121);
    
    // Print all values
    Serial.print(sharp_dist_left);
    Serial.print("\n");

    // Delay for a bit before reading the sensor again
    delay(500);
}
