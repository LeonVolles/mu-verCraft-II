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
#include <imu.h>
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

// IMU (Fermion 10DOF: ADXL345 + ITG3205 + QMC/VCM5883L + BMP280)
IMU imu(global_ComplementaryFilter_yawAlpha);

// IR Line Sensors
IRSensors irSensors((int)global_PIN_IR_SENSOR_BM,
                    (int)global_PIN_IR_SENSOR_FL,
                    (int)global_PIN_IR_SENSOR_FR,
                    global_IRSensorDistance_a_meters,
                    global_IRSensorDistance_b_meters); // pins and distance a: sides,b: base between sensors for an isosceles triangle

// Battery monitor (Betaflight-style voltage/current sensing via ADC + divider ratios)
BatteryMonitor batteryMonitor;

// Struct+Queue for communication between WIFI-Control and Motor Management
struct ControlSetpoints
{
  float lift;
  float thrust;
  float diffThrust;
  bool motorsEnabled;
  bool liftEnabled;
};
QueueHandle_t g_controlQueue = nullptr;
static ControlSetpoints g_latestSetpoints{0.0f, 0.0f, 0.0f, false, false};

// **************************************************
// DEFINE/DECLARE ALL TASK HANDLES
// **************************************************
// Declare task handles
TaskHandle_t taskH_Blink = NULL; // just for quick testing, delete later

TaskHandle_t taskH_wifi = NULL;            // task: hosting the webserver/website to get user controls
TaskHandle_t taskH_imu = NULL;             // task: reading IMU and applying filtering
TaskHandle_t taskH_motorManagement = NULL; // task: gets Setpoint, calls Mixer, sends DShot-commands to MotorCtrl
TaskHandle_t taskH_irSensors = NULL;       // task: reading IR-Sensors
TaskHandle_t taskH_calcIrSensors = NULL;   // task: calculating angle+perpendicular velocity from IR-Sensor readings
TaskHandle_t taskH_batteryMonitor = NULL;  // task: reading battery voltage/current, warning on low battery and capacity calculation

// **************************************************
// TASK FUNCTIONS THAT ARE CALLED BY SCHEDULER
// **************************************************
void task_blink(void *parameter)
{
  for (;;)
  { // Infinite loop
    digitalWrite(LED_PIN, HIGH);
    // Serial.println("task_blink: LED ON");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms
    digitalWrite(LED_PIN, LOW);
    // Serial.println("task_blink: LED OFF");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Serial.print("task_blink running on core ");
    // Serial.println(xPortGetCoreID());
  }
}

void task_wifiManager(void *parameter)
{
  for (;;)
  {
    // Cleanup websocket clients (Async server handles everything else)
    networkPiloting.loop();

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void task_imu(void *parameter)
{
  (void)parameter;

  // Let other subsystems bring Serial up.
  vTaskDelay(200 / portTICK_PERIOD_MS);

  Serial.printf("[IMU] task running on core %d\n", xPortGetCoreID());
  imu.init();

  Serial.printf("[IMU] ready=%d accel=%d gyro=%d mag=%d baro=%d\n",
                (int)imu.isReady(), (int)imu.hasAccel(), (int)imu.hasGyro(), (int)imu.hasMag(), (int)imu.hasBaro());

  // Yaw complementary filter: mostly gyro, slow magnetometer correction
  imu.setYawFilterAlpha(global_ComplementaryFilter_yawAlpha);
  imu.resetYawToMag();

  // Optional: calibrate accel reference (keep craft still + level for ~1s)
  if (imu.hasAccel())
  {
    Serial.println("[IMU] calibrating accel reference...");
    imu.calibrateAccelReference(200, 5);
    Serial.println("[IMU] accel calibration done");
  }

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = 50 / portTICK_PERIOD_MS; // 20 Hz

  for (;;)
  {
    float ax, ay, az;
    float gx, gy, gz;
    int16_t mx, my, mz;
    float head;
    float tempC, pres;

    imu.getAccel_raw(&ax, &ay, &az);
    imu.getGyro_raw(&gx, &gy, &gz);
    imu.getMag_raw(&mx, &my, &mz, &head);
    imu.getEnv(&tempC, &pres);

    // Update yaw estimate (gyro+mag complementary filter) using the same gyro/mag values we just read
    imu.updateYawComplementaryFrom(gz, head);
    const float yaw_PureCompass = head;
    const float yaw_PureGyro = imu.getYawGyro_deg();
    const float yaw_Complementary = imu.getYaw_deg();

    // Rate-limited debug output (SerialPlot friendly)
    // Serial.printf("ax:%.2f ay:%.2f az:%.2f gx:%.3f gy:%.3f gz:%.3f mx:%d my:%d mz:%d yaw_PureCompass:%.1f yaw_PureGyro:%.1f yaw_Complementary:%.1f t:%.2f p:%.2f\n",
    //               ax, ay, az,
    //               gx, gy, gz,
    //               (int)mx, (int)my, (int)mz,
    //               yaw_PureCompass,
    //               yaw_PureGyro,
    //               yaw_Complementary,
    //               tempC, pres);

    vTaskDelayUntil(&lastWake, period);
  }
}

void task_motorManagement(void *parameter)
{
  MotorMixer *mixer = static_cast<MotorMixer *>(parameter);

  ControlSetpoints mySetPoint{0, 0, 0, false, false};

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
    // Enforce emergency override at MotorCtrl level as well.
    motorCtrl.setMotorsEnabled(mySetPoint.motorsEnabled);

    if (!mySetPoint.motorsEnabled)
    {
      mixer->setLift(0.0f);
      mixer->setDiffThrust(0.0f);
      mixer->setThrust(0.0f);
    }
    else
    {
      // Lift OFF only affects the two front motors.
      if (!mySetPoint.liftEnabled)
      {
        // Hard-stop lift motors and keep mixer lift at 0.
        motorCtrl.applyLiftOff();
        mixer->setLift(0.0f);
      }
      else
      {
        mixer->setLift(mySetPoint.lift);
      }

      // Thrust + steering should still work even if lift is OFF.
      mixer->setDiffThrust(mySetPoint.diffThrust);
      mixer->setThrust(mySetPoint.thrust);
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void task_irSensors(void *parameter)
{
    // Order: index0=BM, index1=FL, index2=FR to match IRSensors detectLine mapping.
    const int kAdcPins[] = {(int)global_PIN_IR_SENSOR_BM, (int)global_PIN_IR_SENSOR_FL, (int)global_PIN_IR_SENSOR_FR};
  uint8_t latestSamples[3];                  // Holds most recent 8-bit readings
  uint32_t latestTimestamps[3];              // Holds most recent timestamps in microseconds
  float latestSamplesF[3];                   // Float copy for queueing

    // Low resolution for maximum throughput; 8-bit keeps conversion short.
    analogReadResolution(8);

    // Read IR sensors and save to internal state
    for (;;)
    {
        // Tight polling loop; avoid extra work inside the hot path.
        // The charging of the ADC Capacitor takes a lot longer than the digitisation itself.
        // So we first chrge and then set the timestamps right after reading to be as accurate as possible.
        latestSamples[0] = analogRead(kAdcPins[0]);
        latestTimestamps[0] = micros();

        latestSamples[1] = analogRead(kAdcPins[1]);
        latestTimestamps[1] = micros();

        latestSamples[2] = analogRead(kAdcPins[2]);
        latestTimestamps[2] = micros();

        // Serial.printf("[task_irSensors] BM: %d (%lu us) | FL: %d (%lu us) | FR: %d (%lu us)\n",
        //           latestSamples[0], (unsigned long)latestTimestamps[0],
        //           latestSamples[1], (unsigned long)latestTimestamps[1],
        //           latestSamples[2], (unsigned long)latestTimestamps[2]);

        //save readings and timestamps to IRSensors queue
        latestSamplesF[0] = (float)latestSamples[0];
        latestSamplesF[1] = (float)latestSamples[1];
        latestSamplesF[2] = (float)latestSamples[2];
        uint32_t timestampCopy[3] = {latestTimestamps[0], latestTimestamps[1], latestTimestamps[2]};

        irSensors.enqueueSample(latestSamplesF, timestampCopy);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void task_calcIrSensors(void *parameter)
{
  // If you need to access the global object, you can just use 'irSensors' directly,
  // or pass it via parameter if you prefer strict encapsulation.
  // Here we use the global object 'irSensors'.

  for (;;)
  {
    // Consume all available samples from the queue and run line detection.
    irSensors.processQueue(global_IRSensor_Threshold, global_IRSensor_Hysteresis);

    const float alpha = irSensors.getAlphaToLine();
    const float vPerp = irSensors.getVelocityPerpToLine();
    Serial.printf("[task_calcIrSensors] alpha=%.3f deg, v_perp=%.3f m/s\n", alpha, vPerp);

    // Adjust cadence as needed; slower than producer is fine because queue decouples rate.
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void task_batteryMonitor(void *parameter)
{
  (void)parameter;

  for (;;)
  {
    batteryMonitor.update();

    const float v = batteryMonitor.getVoltage();
    const float a = batteryMonitor.getCurrent();
    const float mah = batteryMonitor.getMAH();

    Serial.printf("[BAT] core=%d V=%.2fV I=%.2fA used=%.0fmAh\n",
                  xPortGetCoreID(),
                  v,
                  a,
                  mah);

    // Push latest battery telemetry to all connected web clients.
    networkPiloting.sendTelemetry(v, a, mah);

    // Relatively large delta_t is fine; update() integrates using millis().
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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

  // **************************************************
  // MOTOR CONTROLLER INITIALIZATION
  // **************************************************
  // Initialize motors with defined IO pins and motor directions from hovercraft_variables.h
  motorCtrl.init(
      global_PIN_MOTOR_FL,
      global_PIN_MOTOR_FR,
      global_PIN_MOTOR_BL,
      global_PIN_MOTOR_BR,
      global_MotorsReversedFL, // flag, if FL motor is reversed or not
      global_MotorsReversedFR, // flag, if FR motor is reversed or not
      global_MotorsReversedBL, // flag, if BL motor is reversed or not
      global_MotorsReversedBR  // flag, if BR motor is reversed or not
  );

  // Start at 0 throttle (arming-safe)
  delay(20);
  motorCtrl.setAllPercent(0.0f, 0.0f, 0.0f, 0.0f);

  delay(20);

  // Init motor mixer
  motorMixer.init(); // inits internal variables with zeros
  delay(100);

  // **************************************************
  // IR SENSORS INITIALIZATION
  // **************************************************
  // Initialize IR Sensors (attaches interrupts), Ensure the pins are set up correctly inside begin()
  irSensors.begin();

  // **************************************************
  // BATTERY MONITOR INITIALIZATION
  // **************************************************
  // Initialize battery monitor
  batteryMonitor.init(
      (int)global_PIN_BATTERY_VOLTAGE_MONITOR,
      (int)global_PIN_BATTERY_CURRENT_MONITOR,
      global_BatteryVoltage_VoltageDividerRatio,
      global_BatteryCurrent_VoltageDividerRatio,
      global_BatteryCurrent_SensorScaler_AmpsPerVolt);

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

  networkPiloting.setLiftCallback([](float liftPercent)
                                  {
    g_latestSetpoints.lift = liftPercent;
    g_latestSetpoints.liftEnabled = (liftPercent > 0.0f);
    if (g_controlQueue != nullptr)
    {
      xQueueOverwrite(g_controlQueue, &g_latestSetpoints);
    }
    Serial.printf("Lift=%.1f%%\n", liftPercent); });

  networkPiloting.setThrustCallback([](float thrustPercent)
                                    {
    g_latestSetpoints.thrust = thrustPercent;
    if (g_controlQueue != nullptr)
    {
      xQueueOverwrite(g_controlQueue, &g_latestSetpoints);
    }
    Serial.printf("Thrust=%.1f%%\n", thrustPercent); });

  networkPiloting.setSteeringCallback([](float steeringPercent)
                                      {
    g_latestSetpoints.diffThrust = steeringPercent;
    if (g_controlQueue != nullptr)
    {
      xQueueOverwrite(g_controlQueue, &g_latestSetpoints);
    }
    Serial.printf("Steering=%.1f%%\n", steeringPercent); });

  networkPiloting.setArmCallback([](bool enabled)
                                 {
    g_latestSetpoints.motorsEnabled = enabled;
    if (!enabled)
    {
      g_latestSetpoints.lift = 0.0f;
      g_latestSetpoints.thrust = 0.0f;
      g_latestSetpoints.diffThrust = 0.0f;
      g_latestSetpoints.liftEnabled = false;
    }
    if (g_controlQueue != nullptr)
    {
      xQueueOverwrite(g_controlQueue, &g_latestSetpoints);
    }
    Serial.printf("Motors %s\n", enabled ? "ON" : "OFF"); });

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

    // IR Sensors Calculation Task
    xTaskCreatePinnedToCore(
        &task_calcIrSensors,
        "task_calcIrSensors",
        4096, // stack size, needs to be adjusted when code is written
        NULL,
        1,
        &taskH_calcIrSensors,
        1);

  // Battery Monitor Task
  xTaskCreatePinnedToCore(
      &task_batteryMonitor,
      "task_batteryMonitor",
      4096, // stack size, needs to be adjusted when code is written, but probably not too big
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
