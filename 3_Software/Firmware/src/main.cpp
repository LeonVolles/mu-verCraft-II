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

// IR Line Sensors
IRSensors irSensors(IR1_PIN, IR2_PIN, IR3_PIN, global_IRSensorDistance_a_meters); // pins and distance a between sensors for a equilateral triangle

// Struct+Queue for communication between WIFI-Control and Motor Management
struct ControlSetpoints
{
  float lift;
  float thrust;
  float diffThrust;
};
QueueHandle_t g_controlQueue = nullptr;

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
    Serial.println("task_blink: LED ON");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms
    digitalWrite(LED_PIN, LOW);
    Serial.println("task_blink: LED OFF");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.print("task_blink running on core ");
    Serial.println(xPortGetCoreID());
  }
}

void task_wifiManager(void *parameter)
{
  for (;;)
  {
    // Here you would read the latest user inputs
    ControlSetpoints mySetPoint;
    mySetPoint.lift = 40.0f; // replace with actual values from web UI
    mySetPoint.thrust = 25.0f;
    mySetPoint.diffThrust = 10.0f;

    if (g_controlQueue != nullptr)
    {
      xQueueOverwrite(g_controlQueue, &mySetPoint); // always keep newest values
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void task_imu(void *parameter)
{
  for (;;)
  {
    // Placeholder for IMU handling code
    Serial.print("task_imu running on core ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void task_motorManagement(void *parameter)
{
  MotorMixer *mixer = static_cast<MotorMixer *>(parameter);

  ControlSetpoints mySetPoint{0, 0, 0};

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
    mixer->setLift(mySetPoint.lift);
    mixer->setDiffThrust(mySetPoint.diffThrust);
    mixer->setThrust(mySetPoint.thrust);

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void task_irSensors(void *parameter)
{
  // If you need to access the global object, you can just use 'irSensors' directly,
  // or pass it via parameter if you prefer strict encapsulation.
  // Here we use the global object 'irSensors'.

  for (;;)
  {
    // Check if the ISRs have completed a full measurement set
    if (irSensors.hasNewMeasurement())
    {
      float alpha = irSensors.getAlphaToLine();
      float vPerp = irSensors.getVelocityPerpToLine();

      // Clear the flag so we don't read the same event multiple times
      irSensors.consumeNewMeasurement();

      // For now: Debug output.
      // Later: Send this to a Navigation/Control Queue.
      Serial.printf("[IR-Task] Alpha: %.2f deg | V_perp: %.3f m/s\n", alpha, vPerp);
    }

    // Polling interval. Since line crossings are short events handled by ISRs,
    // this task just needs to pick up the results frequently enough not to miss
    // updates if you want to react fast. 10-20ms is may be a starting point, but adapt to IMU so it resets the cumulated yaw drift properly.
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void task_batteryMonitor(void *parameter)
{
  for (;;)
  {
    // Placeholder for battery monitoring code
    Serial.print("task_batteryMonitor running on core ");
    Serial.println(xPortGetCoreID());
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

  // Initialize IR Sensors (attaches interrupts), Ensure the pins are set up correctly inside begin()
  irSensors.begin();

  // **************************************************
  // CREATE RTOS QUEUES FOR COMMUNICATION BETWEEN TASKS
  // **************************************************
  g_controlQueue = xQueueCreate(1, sizeof(ControlSetpoints)); // length 1

  if (g_controlQueue == nullptr)
  {
    Serial.println("Failed to create control queue");
  }

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
      4096, // stack size, needs to be adjusted when code is written
      NULL,
      1,
      &taskH_wifi,
      0);

  // IMU Task
  xTaskCreatePinnedToCore(
      &task_imu,
      "task_imu",
      4096, // stack size, needs to be adjusted when code is written
      NULL,
      1,
      &taskH_imu,
      1);

  // Motor Management Task
  xTaskCreatePinnedToCore(
      &task_motorManagement,
      "task_motorManagement",
      4096,        // stack size, may need to be adjusted
      &motorMixer, // pass pointer to MotorMixer
      1,
      &taskH_motorManagement,
      1);

  // IR Sensors Task
  xTaskCreatePinnedToCore(
      &task_irSensors,
      "task_irSensors",
      4096, // smaller stack as not much is done here, this should be fine, to see in future
      NULL,
      2,                // higher priority to stay up to date with line crossings
      &taskH_irSensors, // pass pointer to IR-Sensor function
      1);

  // Battery Monitor Task
  xTaskCreatePinnedToCore(
      &task_batteryMonitor,
      "task_batteryMonitor",
      2048, // stack size, needs to be adjusted when code is written, but probably not too big
      NULL,
      1,
      &taskH_batteryMonitor,
      1);
}

void loop()
{
  // Nothing to do here for now
  //??  Copilot suggested:
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
}
