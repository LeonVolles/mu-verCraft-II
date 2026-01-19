// Motor test code
// Starts the ESC, sets it to be bidirectionnal, then beeps, then makes it go one way then the other over and over at low speeds

#include <Arduino.h>
#include "DShotESC.h"

#define PEAKSPEED 200 // Maximum throttle value to be applied (throttle between -999 and 999)

const int LED_PIN = LED_BUILTIN; // choose a free pin

// To ramp the motor to +-PEAKSPEED
int16_t motorSpeed = 0;    // Variable to hold the current motor speed
int increaseDirection = 1; // Direction of speed increase (1 for increasing, -1 for decreasing)

// Create an Electronic Speed Controller (ESC) object
DShotESC esc0;
DShotESC esc1;
DShotESC esc2;
DShotESC esc3;

void setup()
{
  sleep(2); // Wait for 2 seconds to allow ESC to power up

  Serial.begin(115200); // Initialize serial communication for debugging
  pinMode(LED_PIN, OUTPUT);

  // Set up Motor 1
  esc0.install(GPIO_NUM_43, RMT_CHANNEL_0); // Connect ESC to GPIO pin 43 using RMT channel 0 (motor 1)
  esc0.init();                              // Initialize the ESC
  esc0.setReversed(false);                  // Set normal rotation direction (not reversed)
  esc0.set3DMode(true);                     // Enable bidirectional mode (allows forward and reverse)

  // Set up Motor 2
  esc1.install(GPIO_NUM_8, RMT_CHANNEL_1); // Connect ESC to GPIO pin 8 using RMT channel 1 )(motor 2)
  esc1.init();
  esc1.setReversed(false);
  esc1.set3DMode(true);

  // Set up Motor 3
  esc2.install(GPIO_NUM_7, RMT_CHANNEL_2); // Connect ESC to GPIO pin 7 using RMT channel 2 (motor 3)
  esc2.init();
  esc2.setReversed(false);
  esc2.set3DMode(true);

  // Set up Motor 4
  esc3.install(GPIO_NUM_44, RMT_CHANNEL_3); // Connect ESC to GPIO pin 44 using RMT channel 3 (motor 4)
  esc3.init();
  esc3.setReversed(false);
  esc3.set3DMode(true);

  // Generate a sequence of beeps to indicate ESC is ready
  for (int i = 0; i < 5; i++)
  {
    esc0.beep(i); // Send 5 beep commands with increasing index
    esc1.beep(i);
    esc2.beep(i);
    esc3.beep(i);
  }
}

void loop()
{
  // Ramp motor speed up and down between -PEAKSPEED and +PEAKSPEED
  esc0.sendThrottle3D(motorSpeed); // Send the calculated throttle value to the ESC

  esc1.sendThrottle3D(motorSpeed);

  esc2.sendThrottle3D(motorSpeed);

  esc3.sendThrottle3D(motorSpeed);

  motorSpeed += increaseDirection * 5; // Increment or decrement motor speed

  // Reverse direction if peak speed is reached
  if (motorSpeed >= PEAKSPEED || motorSpeed <= -PEAKSPEED)
  {
    increaseDirection *= -1; // Reverse direction when reaching peak speed
  }

  // Small delay for stability (do not ramp too fast!)
  delay(30);

  // Toggle built-in LED depending on motor direction
  digitalWrite(LED_PIN, motorSpeed > 0 ? HIGH : LOW);
}
