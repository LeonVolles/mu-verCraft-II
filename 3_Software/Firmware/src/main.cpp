#include <Arduino.h>

#include <math.h>

// RTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Reset reason / diagnostics
#include "esp_system.h"
#include "esp_heap_caps.h"

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
#include <heading_controller.h>
#include <camera_placeholder.h>
#include <autonomous_sequence.h>

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

// Autonomous "M" mode sequencer (time-based heading/thrust script)
AutonomousSequence autonomousSequence;

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
static portMUX_TYPE g_setpointsMux = portMUX_INITIALIZER_UNLOCKED;

// Shared IMU data for controllers (updated by IMU task, consumed by motor/control task)
static volatile float g_yawRateMeasured_dps = 0.0f;  // yaw rate (deg/s), sign consistent with IMU yaw integration
static volatile uint32_t g_lastYawRateUpdate_us = 0; // micros() timestamp

// Shared yaw heading estimate from the complementary filter (deg, typically 0..360).
static volatile float g_yawMeasured_deg = 0.0f;
static volatile uint32_t g_lastYawUpdate_us = 0; // micros() timestamp

// Battery telemetry is produced by the battery task and sent by the wifi task.
static portMUX_TYPE g_telemetryMux = portMUX_INITIALIZER_UNLOCKED;
static float g_battVoltage_V = 0.0f;
static float g_battCurrent_A = 0.0f;
static float g_battUsed_mAh = 0.0f;
static bool g_battTelemetryPending = false;

// RTC retained diagnostics (helps when the board keeps rebooting).
RTC_DATA_ATTR static uint32_t g_bootCount = 0;

// Runtime-tunable PID gains (editable via /pid without reflashing).
// These are initialized from the compile-time defaults in hovercraft_variables.
struct RuntimePidTunings
{
  float yawKp;
  float yawKi;
  float yawKd;
  float headingKp;
  float headingKi;
  float headingKd;
};

static portMUX_TYPE g_pidMux = portMUX_INITIALIZER_UNLOCKED;
static RuntimePidTunings g_pidTunings{
    global_YawRatePid_Kp,
    global_YawRatePid_Ki,
    global_YawRatePid_Kd,
    global_HeadingPid_Kp,
    global_HeadingPid_Ki,
    global_HeadingPid_Kd};
static bool g_pidTuningsDirty = false;

// New mode: heading hold (triggered via web UI "M" button).
static volatile bool g_headingHoldActive = false;
static volatile float g_headingTarget_deg = 0.0f;

static inline float wrap360_local(float deg)
{
  if (!isfinite(deg))
    return 0.0f;
  while (deg >= 360.0f)
    deg -= 360.0f;
  while (deg < 0.0f)
    deg += 360.0f;
  return deg;
}

static void setHeading(float heading_deg)
{
  g_headingTarget_deg = wrap360_local(heading_deg);
  g_headingHoldActive = true;
}

static void cancelHeadingHold()
{
  g_headingHoldActive = false;
}

static inline void queueLatestSetpointsSnapshot()
{
  if (g_controlQueue == nullptr)
  {
    return;
  }
  ControlSetpoints snap;
  portENTER_CRITICAL(&g_setpointsMux);
  snap = g_latestSetpoints;
  portEXIT_CRITICAL(&g_setpointsMux);
  xQueueOverwrite(g_controlQueue, &snap);
}

static inline const char *resetReasonToString(esp_reset_reason_t r)
{
  switch (r)
  {
  case ESP_RST_UNKNOWN:
    return "UNKNOWN";
  case ESP_RST_POWERON:
    return "POWERON";
  case ESP_RST_EXT:
    return "EXT";
  case ESP_RST_SW:
    return "SW";
  case ESP_RST_PANIC:
    return "PANIC";
  case ESP_RST_INT_WDT:
    return "INT_WDT";
  case ESP_RST_TASK_WDT:
    return "TASK_WDT";
  case ESP_RST_WDT:
    return "WDT";
  case ESP_RST_DEEPSLEEP:
    return "DEEPSLEEP";
  case ESP_RST_BROWNOUT:
    return "BROWNOUT";
  case ESP_RST_SDIO:
    return "SDIO";
  default:
    return "(other)";
  }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  // Note: stack overflows often corrupt state; keep this minimal.
  Serial.printf("[FREERTOS] Stack overflow in task: %s\n", pcTaskName ? pcTaskName : "(null)");
  Serial.flush();
  esp_restart();
}

extern "C" void vApplicationMallocFailedHook(void)
{
  Serial.println("[FREERTOS] Malloc failed");
  Serial.printf("[HEAP] free=%u min=%u\n", (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
  Serial.flush();
  esp_restart();
}

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
  static uint32_t lastDiagMs = 0;

  for (;;)
  {
    const uint32_t nowMs = millis();

    // Periodic diagnostics (helps identify stack/heap pressure leading to WDT/panic).
    if ((nowMs - lastDiagMs) >= 5000)
    {
      lastDiagMs = nowMs;
      const uint32_t freeHeap = (uint32_t)esp_get_free_heap_size();
      const uint32_t minHeap = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
      const uint32_t stMotor = (uint32_t)uxTaskGetStackHighWaterMark(taskH_motorManagement) * (uint32_t)sizeof(StackType_t);
      const uint32_t stImu = (uint32_t)uxTaskGetStackHighWaterMark(taskH_imu) * (uint32_t)sizeof(StackType_t);
      const uint32_t stWifi = (uint32_t)uxTaskGetStackHighWaterMark(taskH_wifi) * (uint32_t)sizeof(StackType_t);
      const uint32_t stBat = (uint32_t)uxTaskGetStackHighWaterMark(taskH_batteryMonitor) * (uint32_t)sizeof(StackType_t);
      Serial.printf("[DIAG] heap_free=%lu heap_min=%lu stackB motor=%lu imu=%lu wifi=%lu bat=%lu\n",
                    (unsigned long)freeHeap,
                    (unsigned long)minHeap,
                    (unsigned long)stMotor,
                    (unsigned long)stImu,
                    (unsigned long)stWifi,
                    (unsigned long)stBat);
    }

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
    vTaskDelay(2000 / portTICK_PERIOD_MS); // wait 2 s before starting calibration
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
    // Note: We keep the existing yaw-rate sign convention used by the inner yaw-rate controller.
    // (It may differ from the yaw angle integration sign; the controller/mixer convention wins here.)
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

      // For debugging:
      // print magnetometer-based heading, gyro-Integrated heading and complementary filter results#
      // Serial.printf("[IMU] Mag Heading=%.1f deg, Gyro Integrated Yaw=%.1f deg, Comp Filter Yaw=%.1f deg\n",
      //               heading_deg,
      //               imu.getYawGyro_deg(),
      //               imu.getYaw_deg());

      // Mag, Gyro, Comp
      // Serial.printf("%0.2f,%0.2f,%0.2f\n", heading_deg, imu.getYawGyro_deg(), imu.getYaw_deg());
    }

    imu.updateYawComplementaryFrom(gz, heading_deg);

    // Share heading estimate for controllers.
    const float yaw_deg = imu.getYaw_deg();
    if (isfinite(yaw_deg))
    {
      g_yawMeasured_deg = yaw_deg;
      g_lastYawUpdate_us = micros();
    }

    if ((millis() - lastPrintMs) >= 200)
    {
      lastPrintMs = millis();
      // Serial.printf("[IMU] yawRate=%.1f dps yaw=%.1f deg\n", yawRate_dps, imu.getYaw_deg());
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

void task_motorManagement(void *parameter)
{
  MotorMixer *mixer = static_cast<MotorMixer *>(parameter);

  auto yawRateSetpointFromSteeringPercent_dps = [](float steeringPercent) -> float
  {
    // Betaflight-style "Actual Rates" inspired mapping.
    // Input: steeringPercent in [-100..100]. Output: yaw-rate setpoint in deg/s.
    const float stick = constrain(steeringPercent, -100.0f, 100.0f) * 0.01f; // -1..1
    const float a = fabsf(stick);

    const float maxRate_dps = fmaxf(0.0f, global_MaxYawRateSetpoint_dps);
    float center_dps = fmaxf(0.0f, global_YawCenterSensitivity);
    if (center_dps > maxRate_dps)
      center_dps = maxRate_dps;

    const float expo = constrain(global_YawRateExpo, 0.0f, 1.0f);
    const float p = 3.0f + 2.0f * expo; // expo=0 -> cubic, expo=1 -> quintic

    const float rateAbs_dps = center_dps * a + (maxRate_dps - center_dps) * powf(a, p);
    return (stick < 0.0f) ? -rateAbs_dps : rateAbs_dps;
  };

  PIDController yawRatePid;
  HeadingController headingPid;

  auto applyPidTunings = [&](const RuntimePidTunings &t)
  {
    yawRatePid.init(
        t.yawKp,
        t.yawKi,
        t.yawKd,
        global_YawRatePid_OutputLimit,
        global_YawRatePid_IntegratorLimit);

    headingPid.init(
        t.headingKp,
        t.headingKi,
        t.headingKd,
        global_HeadingPid_OutputLimit_dps,
        global_HeadingPid_IntegratorLimit_dps);
  };

  RuntimePidTunings localTunings;
  portENTER_CRITICAL(&g_pidMux);
  localTunings = g_pidTunings;
  portEXIT_CRITICAL(&g_pidMux);
  applyPidTunings(localTunings);

  bool lastHeadingHoldActive = false;
  bool lastMotorsEnabled = false;
  bool lastLiftEnabledWhileArmed = false;

  ControlSetpoints mySetPoint{0, 0, 0, false, false};

  // Control loop timing
  // Use microsecond scheduling to support fractional millisecond periods
  // (e.g. 400 Hz -> 2500 us). vTaskDelayUntil() works in ticks and would
  // truncate to whole milliseconds.
  const float controlRateHz = fmaxf(1.0f, global_ControlLoopRate_Hz);
  const uint32_t controlPeriodUs = (uint32_t)lroundf(1e6f / controlRateHz);
  uint32_t nextWakeUs = micros();
  uint32_t lastUpdateUs = nextWakeUs;

  for (;;)
  {
    // Apply PID tuning updates (from /pid) at a safe point.
    bool pidDirty = false;
    portENTER_CRITICAL(&g_pidMux);
    pidDirty = g_pidTuningsDirty;
    if (pidDirty)
    {
      localTunings = g_pidTunings;
      g_pidTuningsDirty = false;
    }
    portEXIT_CRITICAL(&g_pidMux);
    if (pidDirty)
    {
      applyPidTunings(localTunings);
      Serial.printf("[PID] updated yaw(kp=%.4f ki=%.4f kd=%.4f) heading(kp=%.4f ki=%.4f kd=%.4f)\n",
                    (double)localTunings.yawKp,
                    (double)localTunings.yawKi,
                    (double)localTunings.yawKd,
                    (double)localTunings.headingKp,
                    (double)localTunings.headingKi,
                    (double)localTunings.headingKd);
    }

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

    // Work on a copy so autonomous overrides don't permanently mutate the last user setpoints.
    ControlSetpoints appliedSetpoints = mySetPoint;

    // Battery safety gate: if cutoff is active, force motors OFF regardless of user arming.
    const bool effectiveMotorsEnabled = appliedSetpoints.motorsEnabled && !g_lowBatteryMotorCutoff;

    // Update autonomous sequence (M-Mode). This may override thrust and heading.
    const uint32_t lastYawUs = g_lastYawUpdate_us;
    const bool yawFresh = (lastYawUs != 0) && ((nowUs - lastYawUs) < 150000); // 150ms freshness
    const float yawMeasured_deg = yawFresh ? g_yawMeasured_deg : 0.0f;

    autonomousSequence.update(millis(), yawMeasured_deg, yawFresh, effectiveMotorsEnabled);

    if (autonomousSequence.isActive())
    {
      // During M-Mode we override user commands.
      // - Calibration must keep all motors at 0 output for stability.
      // - After calibration, thrust is overridden by the sequence.
      const AutonomousSequence::State s = autonomousSequence.state();
      const bool holdMotorsAtZero = (s == AutonomousSequence::State::Calibrating) || (s == AutonomousSequence::State::WaitingForArm);

      if (holdMotorsAtZero)
      {
        appliedSetpoints.thrust = 0.0f;
        appliedSetpoints.diffThrust = 0.0f;
        appliedSetpoints.lift = 0.0f;
        appliedSetpoints.liftEnabled = false;
      }
      else
      {
        if (autonomousSequence.overrideThrust())
        {
          appliedSetpoints.thrust = constrain(autonomousSequence.thrustOverride_percent(), -100.0f, 100.0f);
        }
        appliedSetpoints.diffThrust = 0.0f;
      }

      if (autonomousSequence.wantsHeadingHold())
      {
        setHeading(autonomousSequence.headingTarget_deg());
      }
    }

    // Apply whatever values are in mySetPoint (last known setpoints)
    // Enforce emergency override at MotorCtrl level as well (edge-triggered to avoid spamming DShot).
    if (effectiveMotorsEnabled != lastMotorsEnabled)
    {
      motorCtrl.setMotorsEnabled(effectiveMotorsEnabled);
      lastMotorsEnabled = effectiveMotorsEnabled;

      // When disarming, also clear lift-edge tracking.
      if (!effectiveMotorsEnabled)
      {
        lastLiftEnabledWhileArmed = false;
      }
    }

    // Compute yaw rate setpoint from steering input.
    // `diffThrust` coming from the app is interpreted as yaw-rate command in percent [-100..100].
    float yawRateSetpoint_dps = yawRateSetpointFromSteeringPercent_dps(appliedSetpoints.diffThrust);

    const uint32_t lastGyroUs = g_lastYawRateUpdate_us;
    const bool gyroFresh = (lastGyroUs != 0) && ((nowUs - lastGyroUs) < 50000); // 50ms freshness
    const float yawRateMeasured_dps = gyroFresh ? g_yawRateMeasured_dps : 0.0f;

    const bool headingHold = g_headingHoldActive;
    if (headingHold && !lastHeadingHoldActive)
    {
      headingPid.reset();
    }
    lastHeadingHoldActive = headingHold;

    // yawFresh/yawMeasured_deg computed above (also used by autonomous sequencer)

    if (headingHold)
    {
      // Outer loop: heading error -> yaw-rate setpoint (deg/s).
      // Keep the inner loop unchanged.
      if (yawFresh && gyroFresh)
      {
        yawRateSetpoint_dps = headingPid.update(g_headingTarget_deg, yawMeasured_deg, yawRateMeasured_dps, dt_s);
      }
      else
      {
        headingPid.reset();
        yawRateSetpoint_dps = 0.0f;
      }
    }

    // If the sequence finishes or is aborted, force-stop motors and exit M-Mode.
    if (autonomousSequence.consumeExitRequest())
    {
      cancelHeadingHold();

      portENTER_CRITICAL(&g_setpointsMux);
      g_latestSetpoints.motorsEnabled = false;
      g_latestSetpoints.liftEnabled = false;
      g_latestSetpoints.lift = 0.0f;
      g_latestSetpoints.thrust = 0.0f;
      g_latestSetpoints.diffThrust = 0.0f;
      portEXIT_CRITICAL(&g_setpointsMux);

      queueLatestSetpointsSnapshot();

      motorCtrl.setMotorsEnabled(false);
      mixer->setLiftThrustDiff(0.0f, 0.0f, 0.0f);

      Serial.println("[AUTO] sequence exit -> motors OFF, M-Mode OFF");
    }

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

    // Apply mixer outputs once per loop to reduce DShot/RMT load.
    float liftCmd = 0.0f;
    float thrustCmd = 0.0f;
    float diffCmdToMixer = 0.0f;

    if (!effectiveMotorsEnabled)
    {
      // Keep the selected lift setpoint latched even while disarmed,
      // so arming immediately applies the preselected lift percentage.
      liftCmd = appliedSetpoints.liftEnabled ? appliedSetpoints.lift : 0.0f;
      thrustCmd = 0.0f;
      diffCmdToMixer = 0.0f;
    }
    else
    {
      // Lift OFF only affects the two front motors.
      const bool liftEnabled = appliedSetpoints.liftEnabled;
      if (!liftEnabled && lastLiftEnabledWhileArmed)
      {
        // Falling edge: hard-stop lift motors immediately.
        motorCtrl.applyLiftOff();
      }
      lastLiftEnabledWhileArmed = liftEnabled;

      liftCmd = liftEnabled ? appliedSetpoints.lift : 0.0f;
      thrustCmd = appliedSetpoints.thrust;
      diffCmdToMixer = diffCmd;
    }

    mixer->setLiftThrustDiff(liftCmd, thrustCmd, diffCmdToMixer);

    // Periodic scheduling with sub-millisecond resolution.
    nextWakeUs += controlPeriodUs;
    int32_t remainingUs = (int32_t)(nextWakeUs - micros());

    // If we're far behind (e.g. due to WiFi spikes), resync to avoid long catch-up.
    if (remainingUs < -(int32_t)(controlPeriodUs))
    {
      nextWakeUs = micros();
      remainingUs = (int32_t)controlPeriodUs;
    }

    // Coarse sleep in whole milliseconds.
    if (remainingUs > 1500)
    {
      vTaskDelay(pdMS_TO_TICKS((uint32_t)(remainingUs / 1000)));
    }

    // Fine sleep for the last < ~1ms.
    remainingUs = (int32_t)(nextWakeUs - micros());
    if (remainingUs > 0)
    {
      delayMicroseconds((uint32_t)remainingUs);
    }
  }
}

void task_irSensors(void *parameter)
{
  // Order: index0=BM, index1=FL, index2=FR to match IRSensors detectLine mapping.
  const int kAdcPins[] = {(int)global_PIN_IR_SENSOR_BM, (int)global_PIN_IR_SENSOR_FL, (int)global_PIN_IR_SENSOR_FR};
  uint8_t latestSamples[3];     // Holds most recent 8-bit readings
  uint32_t latestTimestamps[3]; // Holds most recent timestamps in microseconds

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

    irSensors.enqueueSample(latestSamples, latestTimestamps);

        vTaskDelay(2 / portTICK_PERIOD_MS);
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
    // Serial.printf("[task_calcIrSensors] alpha=%.3f deg, v_perp=%.3f m/s\n", alpha, vPerp);

    // Adjust cadence as needed; slower than producer is fine because queue decouples rate.
    vTaskDelay(500 / portTICK_PERIOD_MS);
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

    // Serial.printf("[BAT] core=%d V=%.2fV I=%.2fA used=%.0fmAh\n",
    //    xPortGetCoreID(),
    //    v,
    //    a,
    //    mah);

    // Store latest battery telemetry; wifi task will send it.
    portENTER_CRITICAL(&g_telemetryMux);
    g_battVoltage_V = v;
    g_battCurrent_A = a;
    g_battUsed_mAh = mah;
    g_battTelemetryPending = true;
    portEXIT_CRITICAL(&g_telemetryMux);

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

      portENTER_CRITICAL(&g_setpointsMux);
      g_latestSetpoints.motorsEnabled = false;
      g_latestSetpoints.liftEnabled = false;
      g_latestSetpoints.lift = 0.0f;
      g_latestSetpoints.thrust = 0.0f;
      g_latestSetpoints.diffThrust = 0.0f;
      portEXIT_CRITICAL(&g_setpointsMux);

      queueLatestSetpointsSnapshot();

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

  // Print reset reason early to distinguish WDT/brownout/panic.
  const esp_reset_reason_t rr = esp_reset_reason();
  g_bootCount++;
  Serial.printf("[BOOT] boots=%lu reset_reason=%d (%s)\n", (unsigned long)g_bootCount, (int)rr, resetReasonToString(rr));

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
                                    portENTER_CRITICAL(&g_setpointsMux);
                                    g_latestSetpoints.lift = liftPercent;
                                    g_latestSetpoints.liftEnabled = (liftPercent > 0.0f);
                                    portEXIT_CRITICAL(&g_setpointsMux);
                                    queueLatestSetpointsSnapshot();
                                    // Serial.printf("Lift=%.1f%%\n", liftPercent);
                                  });

  networkPiloting.setThrustCallback([](float thrustPercent)
                                    {
                                      // Scale thrust so the web UI's full deflection (100%) maps to a configurable max thrust.
                                      const float scaled = thrustPercent * (global_WebThrustPresetPercent / 100.0f);
                                      portENTER_CRITICAL(&g_setpointsMux);
                                      g_latestSetpoints.thrust = constrain(scaled, -100.0f, 100.0f);
                                      portEXIT_CRITICAL(&g_setpointsMux);
                                      queueLatestSetpointsSnapshot();
                                      // Serial.printf("Thrust=%.1f%%\n", thrustPercent);
                                    });

  networkPiloting.setSteeringCallback([](float steeringPercent)
                                      {
                                        portENTER_CRITICAL(&g_setpointsMux);
                                        g_latestSetpoints.diffThrust = steeringPercent;
                                        portEXIT_CRITICAL(&g_setpointsMux);
                                        queueLatestSetpointsSnapshot();
                                        // Serial.printf("Steering=%.1f%%\n", steeringPercent);
                                      });

  networkPiloting.setArmCallback([](bool enabled)
                                 {
                                   portENTER_CRITICAL(&g_setpointsMux);
                                   g_latestSetpoints.motorsEnabled = enabled;
                                   if (!enabled)
                                   {
                                     g_latestSetpoints.lift = 0.0f;
                                     g_latestSetpoints.thrust = 0.0f;
                                     g_latestSetpoints.diffThrust = 0.0f;
                                     g_latestSetpoints.liftEnabled = false;
                                   }
                                   portEXIT_CRITICAL(&g_setpointsMux);
                                   queueLatestSetpointsSnapshot();
                                   // Serial.printf("Motors %s\n", enabled ? "ON" : "OFF");
                                 });

  networkPiloting.setAutoModeCallback([](bool enabled)
                                      {
    if (enabled)
    {
      // Start autonomous M-Mode sequence (runs inside motor task).
      autonomousSequence.requestStart();
      Serial.println("[AUTO] autoMode ON -> start sequence");
    }
    else
    {
      // Abort sequence at any time.
      autonomousSequence.requestStop();
      Serial.println("[AUTO] autoMode OFF -> abort sequence");
    } });

  // Provide live PID tuning via /pid (no reflashing).
  networkPiloting.setPidGetProvider([](NetworkPiloting::PidTunings &out)
                                    {
                                      portENTER_CRITICAL(&g_pidMux);
                                      out.yawKp = g_pidTunings.yawKp;
                                      out.yawKi = g_pidTunings.yawKi;
                                      out.yawKd = g_pidTunings.yawKd;
                                      out.headingKp = g_pidTunings.headingKp;
                                      out.headingKi = g_pidTunings.headingKi;
                                      out.headingKd = g_pidTunings.headingKd;
                                      portEXIT_CRITICAL(&g_pidMux); });

  networkPiloting.setPidSetHandler([](const NetworkPiloting::PidTunings &in) -> bool
                                   {
                                     // Minimal validation: reject NaN/Inf.
                                     if (!isfinite(in.yawKp) || !isfinite(in.yawKi) || !isfinite(in.yawKd) ||
                                         !isfinite(in.headingKp) || !isfinite(in.headingKi) || !isfinite(in.headingKd))
                                     {
                                       return false;
                                     }

                                     portENTER_CRITICAL(&g_pidMux);
                                     g_pidTunings.yawKp = in.yawKp;
                                     g_pidTunings.yawKi = in.yawKi;
                                     g_pidTunings.yawKd = in.yawKd;
                                     g_pidTunings.headingKp = in.headingKp;
                                     g_pidTunings.headingKi = in.headingKi;
                                     g_pidTunings.headingKd = in.headingKd;
                                     g_pidTuningsDirty = true;
                                     portEXIT_CRITICAL(&g_pidMux);
                                     return true; });

  // Provide /debug endpoint JSON (useful when no serial monitor is attached).
  networkPiloting.setDebugProvider([](char *out, size_t outSize) -> size_t
                                   {
                                    if (out == nullptr || outSize == 0)
                                    {
                                      return 0;
                                    }

                                    ControlSetpoints sp;
                                    portENTER_CRITICAL(&g_setpointsMux);
                                    sp = g_latestSetpoints;
                                    portEXIT_CRITICAL(&g_setpointsMux);

                                    float v = 0.0f, a = 0.0f, mah = 0.0f;
                                    portENTER_CRITICAL(&g_telemetryMux);
                                    v = g_battVoltage_V;
                                    a = g_battCurrent_A;
                                    mah = g_battUsed_mAh;
                                    portEXIT_CRITICAL(&g_telemetryMux);

                                    const float yawDeg = g_yawMeasured_deg;
                                    const bool autoMode = autonomousSequence.isActive();
                                    const bool headingHold = g_headingHoldActive;

                                    const esp_reset_reason_t rr = esp_reset_reason();
                                    const uint32_t freeHeap = (uint32_t)esp_get_free_heap_size();
                                    const uint32_t minHeap = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

                                    auto stackBytes = [](TaskHandle_t h) -> uint32_t
                                    {
                                      if (h == nullptr)
                                      {
                                        return 0;
                                      }
                                      return (uint32_t)uxTaskGetStackHighWaterMark(h) * (uint32_t)sizeof(StackType_t);
                                    };

                                    const uint32_t stMotor = stackBytes(taskH_motorManagement);
                                    const uint32_t stImu = stackBytes(taskH_imu);
                                    const uint32_t stWifi = stackBytes(taskH_wifi);
                                    const uint32_t stBat = stackBytes(taskH_batteryMonitor);

                                    const int n = snprintf(
                                        out,
                                        outSize,
                                        "{"
                                        "\"ok\":true,"
                                        "\"boots\":%lu,"
                                        "\"reset_reason\":{\"code\":%d,\"text\":\"%s\"},"
                                        "\"heap\":{\"free\":%lu,\"min\":%lu},"
                                        "\"stackB\":{\"motor\":%lu,\"imu\":%lu,\"wifi\":%lu,\"bat\":%lu},"
                                        "\"yaw\":{\"deg\":%.1f},"
                                        "\"mode\":{\"autoMode\":%s,\"headingHold\":%s},"
                                        "\"battery\":{\"V\":%.2f,\"A\":%.2f,\"mAh\":%.0f,\"cutoff\":%s},"
                                        "\"setpoints\":{\"lift\":%.1f,\"thrust\":%.1f,\"diff\":%.1f,\"motors\":%s,\"liftEnabled\":%s}"
                                        "}",
                                        (unsigned long)g_bootCount,
                                        (int)rr,
                                        resetReasonToString(rr),
                                        (unsigned long)freeHeap,
                                        (unsigned long)minHeap,
                                        (unsigned long)stMotor,
                                        (unsigned long)stImu,
                                        (unsigned long)stWifi,
                                        (unsigned long)stBat,
                                        (double)yawDeg,
                                        autoMode ? "true" : "false",
                                        headingHold ? "true" : "false",
                                        (double)v,
                                        (double)a,
                                        (double)mah,
                                        g_lowBatteryMotorCutoff ? "true" : "false",
                                        (double)sp.lift,
                                        (double)sp.thrust,
                                        (double)sp.diffThrust,
                                        sp.motorsEnabled ? "true" : "false",
                                        sp.liftEnabled ? "true" : "false");

                                    if (n <= 0)
                                    {
                                      out[0] = '\0';
                                      return 0;
                                    }
                                    if ((size_t)n >= outSize)
                                    {
                                      out[outSize - 1] = '\0';
                                      return outSize - 1;
                                    }
                                    return (size_t)n; });

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
      3,           // higher priority for motor control
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
  // Nothing to do here
}
