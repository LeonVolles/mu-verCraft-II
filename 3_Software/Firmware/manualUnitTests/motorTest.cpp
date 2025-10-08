// // Motor test code
// // Starts the ESC, sets it to be bidirectionnal, then beeps, then makes it go one way then the other over and over at low speeds

// #include <Arduino.h>
// #include "DShotESC.h"

#define PEAKSPEED 500 // Maximum throttle value to be applied (throttle between -999 and 999)

const int LED_PIN = LED_BUILTIN; // choose a free pin

// // To ramp the motor to +-PEAKSPEED
// int16_t motorSpeed = 0;    // Variable to hold the current motor speed
// int increaseDirection = 1; // Direction of speed increase (1 for increasing, -1 for decreasing)

// // Create an Electronic Speed Controller (ESC) object
// DShotESC esc0;

void setup()
{
  // Serial.begin(115200); // Initialize serial communication for debugging
  pinMode(LED_PIN, OUTPUT);
  esc0.install(GPIO_NUM_1, RMT_CHANNEL_0); // Connect ESC to GPIO pin 10 using RMT channel 0
  esc0.init();                             // Initialize the ESC
  esc0.setReversed(false);                 // Set normal rotation direction (not reversed)
  esc0.set3DMode(true);                    // Enable bidirectional mode (allows forward and reverse)

//   // Generate a sequence of beeps to indicate ESC is ready
//   for (int i = 0; i < 5; i++)
//   {
//     esc0.beep(i); // Send 5 beep commands with increasing index
//   }
// }

// void loop()
// {
//   // Ramp motor speed up and down between -PEAKSPEED and +PEAKSPEED
//   esc0.sendThrottle3D(motorSpeed);     // Send the calculated throttle value to the ESC
//   motorSpeed += increaseDirection * 5; // Increment or decrement motor speed

//   // Reverse direction if peak speed is reached
//   if (motorSpeed >= PEAKSPEED || motorSpeed <= -PEAKSPEED)
//   {
//     increaseDirection *= -1; // Reverse direction when reaching peak speed
//   }

  // Small delay for stability (do not ramp too fast!)
  delay(20);

  // Toggle built-in LED depending on motor direction
  digitalWrite(LED_PIN, motorSpeed > 0 ? HIGH : LOW);
}
