#include <Arduino.h>

// RTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Include file for all global variables
#include "hovercraft_variables.h"

// Include all subsystem headers
#include <motor_ctrl.h>
#include <motor_mixer.h>
// #include <imu.h>       // FOR NOW NOT AT ALL FUNCTIONAL!!!
#include <wifi_manager.h>
#include <network_piloting.h>
#include <ir_sensors.h>
#include <battery_monitor.h>
#include <pid_controller.h>
#include <camera_placeholder.h>

// **************************************************
// DEFINE ALL OBJECTS THAT ARE NEEDED
// **************************************************
// Motor controller with global power scaler, defined in hovercraft_variables.h!!
MotorCtrl motorCtrl(global_AllMotorsScalePercent); // global power scaler reduces the max power to avoid smoking motors, adjust depending on kV of motors

// Mixer that uses the motor controller
MotorMixer motorMixer(motorCtrl); // Gives control mixer access to motor controller

// Network + WiFi helpers
WifiManager wifiManager;
NetworkPiloting networkPiloting;
// http://192.168.4.1/
static const char *AP_SSID = "µ-verCraft-II AP";
static const char *AP_PASSWORD = "Supmicrotech"; // minimum 8 chars for WPA2

// Struct+Queue for communication between WIFI-Control and Motor Management
struct ControlSetpoints
{
  float lift;
  float thrust;
  float diffThrust;
  bool motorsEnabled;
};
QueueHandle_t g_controlQueue = nullptr;
static ControlSetpoints g_latestSetpoints{0.0f, 0.0f, 0.0f, false};

// **************************************************
// DEFINE/DECLARE ALL TASK HANDLES
// **************************************************
// Declare task handles
TaskHandle_t taskH_Blink = NULL; // just for quick testing, delete later

TaskHandle_t taskH_wifi = NULL;            // task: hosting the webserver/website to get user controls
TaskHandle_t taskH_imu = NULL;             // task: reading IMU and applying filtering
TaskHandle_t taskH_motorManagement = NULL; // task: gets Setpoint, calls Mixer, sends DShot-commands to MotorCtrl
TaskHandle_t taskH_irSensors = NULL;       // task: reading IR-Sensors, maybe calculating also rotation relative to the line
TaskHandle_t taskH_batteryMonitor = NULL;  // task: reading battery voltage/current, warning on low battery and capacity calculation

// **************************************************
// (?temporary?) TASK FUNCTIONS THAT ARE CALLED BY SCHEDULER
// **************************************************
void task_blink(void *parameter)
{
  for (;;)
  { // Infinite loop
    digitalWrite(LED_PIN, HIGH);
    //Serial.println("task_blink: LED ON");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms
    digitalWrite(LED_PIN, LOW);
    //Serial.println("task_blink: LED OFF");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //Serial.print("task_blink running on core ");
    //Serial.println(xPortGetCoreID());
  }
}

void task_wifiManager(void *parameter)
{
  for (;;)
  {
    // // Here you would read the latest user inputs
    // ControlSetpoints mySetPoint;
    // mySetPoint.lift = 40.0f; // replace with actual values from web UI
    // mySetPoint.thrust = 25.0f;
    // mySetPoint.diffThrust = 10.0f;

    // if (g_controlQueue != nullptr)
    // {
    //   xQueueOverwrite(g_controlQueue, &mySetPoint); // always keep newest values
    // }

    // Cleanup websocket clients (Async server handles everything else)
    networkPiloting.loop();

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void task_imu(void *parameter)
{
  for (;;)
  {
    // Placeholder for IMU handling code
    //Serial.print("task_imu running on core ");
    //Serial.println(xPortGetCoreID());
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void task_motorManagement(void *parameter)
{
  MotorMixer *mixer = static_cast<MotorMixer *>(parameter);

  ControlSetpoints mySetPoint{0, 0, 0, false};

  for (;;)
  {
    if (g_controlQueue != nullptr)
    {
      // Non blocking receive, timeout 0 ticks
      if (xQueueReceive(g_controlQueue, &mySetPoint, 0) == pdPASS)
      {
        // Got new setpoints
      }
    }

    // Apply whatever values are in mySetPoint (last known setpoints)
    if (!mySetPoint.motorsEnabled)
    {
      mixer->setLift(0.0f);
      mixer->setDiffThrust(0.0f);
      mixer->setThrust(0.0f);
    }
    else
    {
      mixer->setLift(mySetPoint.lift);
      mixer->setDiffThrust(mySetPoint.diffThrust);
      mixer->setThrust(mySetPoint.thrust);
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void task_irSensors(void *parameter)
{
  for (;;)
  {
    // Placeholder for IR sensor handling code
    //Serial.print("task_irSensors running on core ");
    //Serial.println(xPortGetCoreID());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void task_batteryMonitor(void *parameter)
{
  for (;;)
  {
    // Placeholder for battery monitoring code
    //Serial.print("task_batteryMonitor running on core ");
    //Serial.println(xPortGetCoreID());
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

// **************************************************
// SETUP
// **************************************************
void setup()
{
  // Start Serial for debug output
  Serial.begin(115200);

  // Define LED Pin, this is probably a special case, since most other pins are defined in the libary constructors
  pinMode(LED_PIN, OUTPUT);

  // **************************************************
  // INITIALIZE ALL OBJECTS THAT ARE NEEDED
  // **************************************************
  // Initialize motors with defined IO pins and motor directions from hovercraft_variables.h
  motorCtrl.init(
      global_PIN_MOTOR_FL,
      global_PIN_MOTOR_FR,
      global_PIN_MOTOR_BL,
      global_PIN_MOTOR_BR,
      global_MotorsReversedFL,
      global_MotorsReversedFR,
      global_MotorsReversedBL,
      global_MotorsReversedBR);

  // Initialize motor mixer
  motorMixer.init();

  // **************************************************
  // CREATE RTOS QUEUES FOR COMMUNICATION BETWEEN TASKS
  // **************************************************
  g_controlQueue = xQueueCreate(1, sizeof(ControlSetpoints)); // length 1

  if (g_controlQueue == nullptr)
  {
    Serial.println("Failed to create control queue");
  }

  // **************************************************
  // START WIFI ACCESS POINT + WEB CONTROL
  // **************************************************
  wifiManager.startAccessPoint(AP_SSID, AP_PASSWORD);

  networkPiloting.setLiftCallback([](float liftPercent) {
    g_latestSetpoints.lift = liftPercent;
    if (g_controlQueue != nullptr)
    {
      xQueueOverwrite(g_controlQueue, &g_latestSetpoints);
    }
    Serial.printf("Lift=%.1f%%\n", liftPercent);
  });

  networkPiloting.setThrustCallback([](float thrustPercent) {
    g_latestSetpoints.thrust = thrustPercent;
    if (g_controlQueue != nullptr)
    {
      xQueueOverwrite(g_controlQueue, &g_latestSetpoints);
    }
    Serial.printf("Thrust=%.1f%%\n", thrustPercent);
  });

  networkPiloting.setSteeringCallback([](float steeringPercent) {
    g_latestSetpoints.diffThrust = steeringPercent;
    if (g_controlQueue != nullptr)
    {
      xQueueOverwrite(g_controlQueue, &g_latestSetpoints);
    }
    Serial.printf("Steering=%.1f%%\n", steeringPercent);
  });

  networkPiloting.setArmCallback([](bool enabled) {
    g_latestSetpoints.motorsEnabled = enabled;
    if (!enabled)
    {
      g_latestSetpoints.lift = 0.0f;
      g_latestSetpoints.thrust = 0.0f;
      g_latestSetpoints.diffThrust = 0.0f;
    }
    if (g_controlQueue != nullptr)
    {
      xQueueOverwrite(g_controlQueue, &g_latestSetpoints);
    }
    Serial.printf("Motors %s\n", enabled ? "ON" : "OFF");
  });

  networkPiloting.begin();

  // **************************************************
  // CREATE RTOS TASKS FOR EACH SUBSYSTEM
  // **************************************************
  // Test Blink Task
  xTaskCreatePinnedToCore(
      &task_blink,  // Task function
      "task_blink", // Task name
      10000,        // Stack size (bytes)
      NULL,         // Parameters
      1,            // Priority
      &taskH_Blink, // Task handle
      1             // Core 1
  );

  // Wifi Task
  xTaskCreatePinnedToCore(
      &task_wifiManager,
      "task_wifiManager",
      20000,
      NULL,
      1,
      &taskH_wifi,
      0);

  // IMU Task
  xTaskCreatePinnedToCore(
      &task_imu,
      "task_imu",
      20000,
      NULL,
      1,
      &taskH_imu,
      1);

  // Motor Management Task
  xTaskCreatePinnedToCore(
      &task_motorManagement,
      "task_motorManagement",
      20000,
      &motorMixer, // pass pointer to MotorMixer
      1,
      &taskH_motorManagement,
      1);

  // IR Sensors Task
  xTaskCreatePinnedToCore(
      &task_irSensors,
      "task_irSensors",
      20000,
      NULL,
      1,
      &taskH_irSensors,
      1);

  // Battery Monitor Task
  xTaskCreatePinnedToCore(
      &task_batteryMonitor,
      "task_batteryMonitor",
      20000,
      NULL,
      1,
      &taskH_batteryMonitor,
      1);
}

void loop()
{
  // Nothing to do here for now
}
