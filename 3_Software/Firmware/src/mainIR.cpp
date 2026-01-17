/*
  High speed polling on ESP32-S3 (Arduino core v2.0.17 / IDF v4.4.7)
  A0/A1/A2 must be ADC1 pins (typically GPIO 1-10 on S3 boards).
*/

#include <Arduino.h>

constexpr uint8_t kAdcPins[] = {1, 2, 3};  // Map these to your A0/A1/A2
volatile uint8_t latestSamples[3];          // Holds most recent 8-bit readings
volatile uint32_t loopCounter = 0;          // Counts iterations for periodic logging

void setup() {

  // Low resolution for maximum throughput; 8-bit keeps conversion short.
  analogReadResolution(8);
  // Fastest ADC clock divider; lower numbers increase sample rate.
  analogSetClockDiv(1);

  Serial.begin(115200);
}

void pollAdcAndLog() {
  // Tight polling loop; avoid extra work inside the hot path.
  latestSamples[0] = analogRead(kAdcPins[0]);
  latestSamples[1] = analogRead(kAdcPins[1]);
  latestSamples[2] = analogRead(kAdcPins[2]);

  // Lightweight periodic print every 10,000 samples to monitor values.
  if (++loopCounter >= 100000) {
    loopCounter = 0;
    Serial.printf("A0=%u A1=%u A2=%u\n", latestSamples[0], latestSamples[1], latestSamples[2]);
  }
}

void loop() {
  pollAdcAndLog();
}

