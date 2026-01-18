<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## main (application entry point)

### Main idea
This file is the integration point of the whole firmware: it wires together all subsystems (motors, mixer, IMU, WiFi/web control, sensors, battery) and starts the FreeRTOS tasks that make the hovercraft run.

### Where it is used / how it is called
- This is the Arduino entry point compiled by PlatformIO: [src/main.cpp](src/main.cpp).
- `setup()` runs once after boot, then FreeRTOS tasks execute continuously; `loop()` is intentionally unused.

### Project structure: what runs when

#### Boot sequence (high level)
1. **Boot → `setup()`**: initialize peripherals, create the control queue, start WiFi + web control, then create RTOS tasks.
2. **Runtime**: tasks run concurrently on the ESP32’s two cores.
3. **Safety default**: motors start disarmed (`motorsEnabled=false`) and the motor task keeps outputs safe until arming is received.

#### Data flow overview (who talks to whom)
- **Web UI → NetworkPiloting callbacks → control queue**
	- WebSocket messages update `g_latestSetpoints` and immediately publish them using `xQueueOverwrite(g_controlQueue, ...)`.
	- The queue length is 1, so the motor task always consumes the **latest** setpoints.
- **IMU task → shared yaw-rate → motor task**
	- IMU task updates `g_yawRateMeasured_dps` and `g_lastYawRateUpdate_us`.
	- Motor task checks “freshness” (timestamp age) before using yaw-rate for control.
- **Motor task → mixer → motor controller → ESCs**
	- Motor task computes the yaw-rate control command (`diffCmd`) via `PIDController`.
	- `MotorMixer` converts lift/thrust/diff into 4 motor percentages.
	- `MotorCtrl` sends DShot commands to the ESCs.
- **Battery task → NetworkPiloting telemetry**
	- Battery task publishes voltage/current/mAh via `networkPiloting.sendTelemetry(...)`.

#### Task map (what each task does)
The firmware creates multiple tasks in `setup()` using `xTaskCreatePinnedToCore(...)`.

##### task_wifiManager (core 0)
- Calls `networkPiloting.loop()` periodically (every ~50ms).
- Purpose: housekeeping (WebSocket client cleanup); the async server handles network I/O callbacks.

##### task_imu (core 1)
- Initializes IMU, then runs a periodic loop at **200 Hz** (`vTaskDelayUntil(..., 5ms)`).
- Reads gyro continuously; reads magnetometer slower (~20 Hz) and updates yaw complementary filter.
- Publishes yaw rate (deg/s) to the shared variables for the control loop.

##### task_motorManagement (core 1)
- Runs the main control loop at `global_ControlLoopRate_Hz` using `vTaskDelayUntil`.
- Non-blocking reads of `g_controlQueue` to get the latest setpoints.
- Implements yaw-rate control:
	- Converts steering percent to yaw-rate setpoint using `global_MaxYawRateSetpoint_dps`.
	- If motors are armed and gyro data is fresh: `PIDController.update(...)` → `diffCmd`.
	- Otherwise: PID reset + `diffCmd = 0`.
- Applies safety gates:
	- Always calls `motorCtrl.setMotorsEnabled(mySetPoint.motorsEnabled)`.
	- If disarmed, forces thrust and diff to zero; lift can be “latched” but not applied until arming.
	- If lift is disabled, calls `motorCtrl.applyLiftOff()` and forces mixer lift to 0.

##### task_irSensors (core 1)
- Polls for ISR-completed measurements (`irSensors.hasNewMeasurement()`) and consumes them.
- Currently prints debug values (angle to line, lateral velocity). Intended as a future navigation input.
- Typical polling delay is ~20ms.

##### task_batteryMonitor (core 1)
- Calls `batteryMonitor.update()` and prints voltage/current/mAh.
- Sends telemetry to web clients once per second.

##### task_blink (core 1)
- Simple LED blink for testing.

### Functionalities implemented (project-level)
- **System bring-up**: initialize motor outputs, mixer state, sensor interrupts, battery ADC scaling, WiFi AP, and web control.
- **Real-time scheduling**: run IMU acquisition and control loop periodically with `vTaskDelayUntil`.
- **Command & telemetry plumbing**:
	- WebSocket commands → queue → control loop.
	- Battery telemetry → WebSocket broadcast.
- **Core separation**: keep network housekeeping on core 0; keep control/sensing on core 1.
- **Safety behaviors**: disarm defaults, lift-off handling, PID reset on stale sensor data.

### Methods / functions overview
This file is mostly glue code and task entry points (not a class).

- `setup()`
	- Initializes serial + LED, motors (`motorCtrl.init(...)`), mixer (`motorMixer.init()`), IR sensors (`irSensors.begin()`), battery monitor (`batteryMonitor.init(...)`).
	- Creates the control queue (`g_controlQueue = xQueueCreate(1, sizeof(ControlSetpoints))`).
	- Starts WiFi AP and registers NetworkPiloting callbacks (lift/thrust/steering/arm).
	- Calls `networkPiloting.begin()` and creates RTOS tasks.

- `loop()`
	- Empty by design (everything runs in tasks).

- Task functions (`task_wifiManager`, `task_imu`, `task_motorManagement`, `task_irSensors`, `task_batteryMonitor`, `task_blink`)
	- Each is an infinite loop; timing is controlled by `vTaskDelay(...)` or `vTaskDelayUntil(...)`.

### Parameters used (configuration & tuning)
Most parameters come from [src/hovercraft_variables.h](src/hovercraft_variables.h) (with definitions in [src/hovercraft_variables.cpp](src/hovercraft_variables.cpp)). Key groups:

- **Motor hardware & safety**: `global_PIN_MOTOR_*`, `global_MotorsReversed*`, `global_AllMotorsScalePercent`.
- **Control loop**: `global_ControlLoopRate_Hz`, `global_MaxYawRateSetpoint_dps`.
- **Yaw-rate PID**: `global_YawRatePid_Kp`, `global_YawRatePid_Ki`, `global_YawRatePid_Kd`, `global_YawRatePid_OutputLimit`, `global_YawRatePid_IntegratorLimit`.
- **IMU filter**: `global_ComplementaryFilter_yawAlpha`.
- **Web control scaling**: `global_WebThrustPresetPercent`.
- **WiFi**: `global_WifiApSsid`, `global_WifiApPassword`.
- **IR sensors**: `global_PIN_IR_SENSOR_*`, `global_IRSensorDistance_a_meters`.
- **Battery monitor**: `global_PIN_BATTERY_*`, `global_BatteryVoltage_VoltageDividerRatio`, `global_BatteryCurrent_VoltageDividerRatio`, `global_BatteryCurrent_SensorScaler_AmpsPerVolt`.

