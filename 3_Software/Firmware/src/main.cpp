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
  delay(2500);
  imu.init();
  if (!imu.isReady())
  {
    Serial.println("IMU init failed. Check wiring.");
  }
  else
  {
    Serial.println("Calibrating IMU... keep still");
    imu.calibrateAccelReference(500, 5);
    Serial.println("Calibrating IMU... done");
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

    // Corrected acceleration (level frame)
    float axc, ayc, azc;
    imu.getAccel_corrected(&axc, &ayc, &azc);

    // Optional: corrected tilt (deg) to verify close to 0/0 after calibration
    float rc, pc, yc;
    imu.getAngles_corrected(&rc, &pc, &yc);

    // Compact print: A:ax,ay,az G:gx,gy,gz
    Serial.printf("A:%.2f,%.2f,%.2f Ac:%.2f,%.2f,%.2f G:%.3f,%.3f,%.3f Rc:%.1f Pc:%.1f\n", ax, ay, az, axc, ayc, azc, gx, gy, gz, rc, pc);
  }
}