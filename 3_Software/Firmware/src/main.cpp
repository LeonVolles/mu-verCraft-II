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

// RTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Added definitions for the test
const int LED_PIN = LED_BUILTIN;

// Declare task handles
TaskHandle_t taskH_Blink = NULL;

TaskHandle_t taskH_wifi = NULL;
TaskHandle_t taskH_imu = NULL;
TaskHandle_t taskH_motorManagement = NULL; // Setpoint, Mixer, DShot
TaskHandle_t taskH_irSensors = NULL;
TaskHandle_t taskH_batteryMonitor = NULL;


void task_blink(void *parameter) {
  for (;;) { // Infinite loop
    digitalWrite(LED_PIN, HIGH);
    Serial.println("task_blink: LED ON");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms
    digitalWrite(LED_PIN, LOW);
    Serial.println("task_blink: LED OFF");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.print("task_blink running on core ");
    Serial.println(xPortGetCoreID());
  }
}

void task_wifiManager(void *parameter) {
  for (;;) {
    // Placeholder for WiFi management code
    Serial.print("task_wifiManager running on core ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void task_imu(void *parameter) {
  for (;;) {
    // Placeholder for IMU handling code
    Serial.print("task_imu running on core ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void task_motorManagement(void *parameter) {
  for (;;) {
    // Placeholder for motor management code
    Serial.print("task_motorManagement running on core ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void task_irSensors(void *parameter) {
  for (;;) {
    // Placeholder for IR sensor handling code
    Serial.print("task_irSensors running on core ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void task_batteryMonitor(void *parameter) {
  for (;;) {
    // Placeholder for battery monitoring code
    Serial.print("task_batteryMonitor running on core ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Create RTOS Tasks for each subsystem

  // Test Blink Task
  xTaskCreatePinnedToCore(
    &task_blink,         // Task function
    "task_blink",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &taskH_Blink,  // Task handle
    1                  // Core 1
  );

  // Wifi Task
  xTaskCreatePinnedToCore(
    &task_wifiManager,
    "task_wifiManager",
    20000,
    NULL,
    1,
    &taskH_wifi,
    0
  );

  // IMU Task
  xTaskCreatePinnedToCore(
    &task_imu,
    "task_imu",
    20000,
    NULL,
    1,
    &taskH_imu,
    1
  );

  // Motor Management Task
  xTaskCreatePinnedToCore(
    &task_motorManagement,
    "task_motorManagement",
    20000,
    NULL,
    1,
    &taskH_motorManagement,
    1
  );

  // IR Sensors Task
  xTaskCreatePinnedToCore(
    &task_irSensors,
    "task_irSensors",
    20000,
    NULL,
    1,
    &taskH_irSensors,
    1
  );

  // Battery Monitor Task
  xTaskCreatePinnedToCore(
    &task_batteryMonitor,
    "task_batteryMonitor",
    20000,
    NULL,
    1,
    &taskH_batteryMonitor,
    1
  );
}

void loop()
{
  // delay(200);
  // digitalWrite(LED_PIN, HIGH);
  // delay(200);
  // digitalWrite(LED_PIN, LOW);
}