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
// http://192.168.4.1/

// IMU (Fermion 10DOF: ADXL345 + ITG3205 + QMC/VCM5883L + BMP280)
IMU imu(global_ComplementaryFilter_yawAlpha);

// IR Line Sensors
IRSensors irSensors((int)global_PIN_IR_SENSOR_BM,
                    (int)global_PIN_IR_SENSOR_FL,
                    (int)global_PIN_IR_SENSOR_FR,
                    global_IRSensorDistance_a_meters); // pins and distance a between sensors for a equilateral triangle

// Battery monitor (Betaflight-style voltage/current sensing via ADC + divider ratios)
BatteryMonitor batteryMonitor;

// Low-battery safety flags (set by battery task, read by other tasks).
static volatile bool g_lowBatteryLedSolidOn = false;
static volatile bool g_lowBatteryMotorCutoff = false;

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

// Shared IMU data for controllers (updated by IMU task, consumed by motor/control task)
static volatile float g_yawRateMeasured_dps = 0.0f;  // yaw rate (deg/s), sign consistent with IMU yaw integration
static volatile uint32_t g_lastYawRateUpdate_us = 0; // micros() timestamp

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
// TASK FUNCTIONS THAT ARE CALLED BY SCHEDULER
// **************************************************
void task_blink(void *parameter)
{
  for (;;)
  { // Infinite loop
    if (g_lowBatteryLedSolidOn)
    {
      // Low battery indicator: keep LED solid ON.
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      continue;
    }

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

  // Run gyro integration fast for stable rate control.
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t period = pdMS_TO_TICKS(5); // 200 Hz
  if (period == 0)
    period = 1;

  uint32_t lastMagMs = 0;
  uint32_t lastPrintMs = 0;

  for (;;)
  {
    float gx = NAN, gy = NAN, gz = NAN;
    imu.getGyro_raw(&gx, &gy, &gz);

    // Convert to yaw rate in deg/s.
    // Note: IMU yaw integration uses yaw = yaw + gz_deg_s*dt, so yaw_rate (deg/s) consistent with yaw is gz.
    const float yawRate_dps = gz * (180.0f / PI);
    if (isfinite(yawRate_dps))
    {
      g_yawRateMeasured_dps = yawRate_dps;
      g_lastYawRateUpdate_us = micros();
    }

    // Read magnetometer at a slower rate to reduce I2C load; integrate gyro every loop.
    float heading_deg = NAN;
    if (imu.hasMag() && (millis() - lastMagMs) >= 50)
    {
      int16_t mx, my, mz;
      float head;
      imu.getMag_raw(&mx, &my, &mz, &head);
      heading_deg = head;
      lastMagMs = millis();
    }

    imu.updateYawComplementaryFrom(gz, heading_deg);

    if ((millis() - lastPrintMs) >= 200)
    {
      lastPrintMs = millis();
      Serial.printf("[IMU] yawRate=%.1f dps yaw=%.1f deg\n", yawRate_dps, imu.getYaw_deg());
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

void task_motorManagement(void *parameter)
{
  MotorMixer *mixer = static_cast<MotorMixer *>(parameter);

  PIDController yawRatePid;
  yawRatePid.init(
      global_YawRatePid_Kp,
      global_YawRatePid_Ki,
      global_YawRatePid_Kd,
      global_YawRatePid_OutputLimit,
      global_YawRatePid_IntegratorLimit);

  ControlSetpoints mySetPoint{0, 0, 0, false, false};

  TickType_t lastWake = xTaskGetTickCount();
  const uint32_t controlPeriodMs = (uint32_t)(1000.0f / global_ControlLoopRate_Hz);
  TickType_t controlPeriodTicks = pdMS_TO_TICKS(controlPeriodMs);
  if (controlPeriodTicks == 0)
    controlPeriodTicks = 1;
  uint32_t lastUpdateUs = micros();

  for (;;)
  {
    const uint32_t nowUs = micros();
    float dt_s = (nowUs - lastUpdateUs) * 1e-6f;
    lastUpdateUs = nowUs;
    if (!isfinite(dt_s) || dt_s < 0.001f)
      dt_s = 0.001f;
    if (dt_s > 0.05f)
      dt_s = 0.05f;

    if (g_controlQueue != nullptr)
    {
      // Non blocking receive, timeout 0 ticks
      if (xQueueReceive(g_controlQueue, &mySetPoint, 0) == pdPASS)
      {
        // Got new setpoints
      }
    }

    // Battery safety gate: if cutoff is active, force motors OFF regardless of user arming.
    const bool effectiveMotorsEnabled = mySetPoint.motorsEnabled && !g_lowBatteryMotorCutoff;

    // Apply whatever values are in mySetPoint (last known setpoints)
    // Enforce emergency override at MotorCtrl level as well.
    motorCtrl.setMotorsEnabled(effectiveMotorsEnabled);

    // Compute yaw rate setpoint from steering input.
    // `diffThrust` coming from the app is interpreted as yaw-rate command in percent [-100..100].
    const float yawRateSetpoint_dps = (constrain(mySetPoint.diffThrust, -100.0f, 100.0f) / 100.0f) * global_MaxYawRateSetpoint_dps;

    const uint32_t lastGyroUs = g_lastYawRateUpdate_us;
    const bool gyroFresh = (lastGyroUs != 0) && ((nowUs - lastGyroUs) < 50000); // 50ms freshness
    const float yawRateMeasured_dps = gyroFresh ? g_yawRateMeasured_dps : 0.0f;

    float diffCmd = 0.0f;
    if (effectiveMotorsEnabled && gyroFresh)
    {
      diffCmd = yawRatePid.update(yawRateSetpoint_dps, yawRateMeasured_dps, dt_s);
      diffCmd = constrain(diffCmd, -100.0f, 100.0f);
    }
    else
    {
      yawRatePid.reset();
      diffCmd = 0.0f;
    }

    if (!effectiveMotorsEnabled)
    {
      // Keep the selected lift setpoint latched even while disarmed,
      // so arming immediately applies the preselected lift percentage.
      mixer->setLift(mySetPoint.liftEnabled ? mySetPoint.lift : 0.0f);
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

      // Thrust + yaw-rate control should still work even if lift is OFF.
      mixer->setDiffThrust(diffCmd);
      mixer->setThrust(mySetPoint.thrust);
    }

    vTaskDelayUntil(&lastWake, controlPeriodTicks);
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
  (void)parameter;

  uint16_t cutoffSampleCount = 0;
  bool cutoffLatched = false;

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

    // Low battery indicator: LED stays solid ON below the configured warning threshold.
    g_lowBatteryLedSolidOn = (v > 0.0f) && (v < global_BatteryVoltageLow_WarningLow);

    // Motor cutoff: if voltage is below cutoff for more than N samples, force motors OFF.
    if ((v > 0.0f) && (v < global_BatteryVoltageLow_MotorCutoffLow))
    {
      if (cutoffSampleCount < 0xFFFF)
      {
        cutoffSampleCount++;
      }
    }
    else
    {
      cutoffSampleCount = 0;
    }

    const bool cutoffNow = cutoffSampleCount > global_BatteryVoltageLow_MotorCutoffSamples;
    g_lowBatteryMotorCutoff = cutoffNow;

    // On first entry into cutoff, actively disarm and reset setpoints so recovery doesn't auto-spin motors.
    if (cutoffNow && !cutoffLatched)
    {
      cutoffLatched = true;

      g_latestSetpoints.motorsEnabled = false;
      g_latestSetpoints.liftEnabled = false;
      g_latestSetpoints.lift = 0.0f;
      g_latestSetpoints.thrust = 0.0f;
      g_latestSetpoints.diffThrust = 0.0f;

      if (g_controlQueue != nullptr)
      {
        xQueueOverwrite(g_controlQueue, &g_latestSetpoints);
      }

      motorCtrl.setMotorsEnabled(false);
      Serial.println("[BAT] Low voltage cutoff: motors disabled");
    }

    if (!cutoffNow)
    {
      cutoffLatched = false;
    }

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
  wifiManager.startAccessPoint(global_WifiApSsid, global_WifiApPassword);

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
    // Scale thrust so the web UI's full deflection (100%) maps to a configurable max thrust.
    const float scaled = thrustPercent * (global_WebThrustPresetPercent / 100.0f);
    g_latestSetpoints.thrust = constrain(scaled, -100.0f, 100.0f);
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
