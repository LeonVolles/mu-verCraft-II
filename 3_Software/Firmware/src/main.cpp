#include <Arduino.h>

// Include all subsystem headers
#include <motor_ctrl.h>
#include <imu.h>
#include <wifi_manager.h>
#include <network_piloting.h>
#include <ir_sensors.h>
#include <battery_monitor.h>
#include <pid_controller.h>
#include <camera_placeholder.h>

// Added definitions for the test
const int LED_PIN = LED_BUILTIN;

// Global IMU instance
IMU imu;

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(50);
  imu.init();
  if (!imu.isReady())
  {
    Serial.println("IMU init failed. Check wiring.");
  }
}

void loop()
{
  static uint32_t last = 0;
  const uint32_t period_ms = 100; // 10 Hz
  if (millis() - last >= period_ms)
  {
    last = millis();

    float ax, ay, az, gx, gy, gz;
    imu.getAccel_raw(&ax, &ay, &az);
    imu.getGyro_raw(&gx, &gy, &gz);

    // Compact print: A:ax,ay,az G:gx,gy,gz
    Serial.printf("A:%.2f,%.2f,%.2f G:%.3f,%.3f,%.3f\n", ax, ay, az, gx, gy, gz);
  }
}