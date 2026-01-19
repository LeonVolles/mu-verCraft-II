<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## hovercraft_variables (global project settings)

### Main idea
This file is the single place to tune the hovercraft: pin assignments, control loop tuning, and user-facing presets are centralized here so you don’t have to hunt through every subsystem.

### Where it is used / how it is called
- Declarations live in [src/hovercraft_variables.h](src/hovercraft_variables.h) and the actual values are defined in [src/hovercraft_variables.cpp](src/hovercraft_variables.cpp).
- Almost every subsystem consumes these values indirectly, but the main wiring is done in [src/main.cpp](src/main.cpp) when objects are constructed and initialized.

### Define all IO-Pins 
These constants define how the ESP32 is wired to the hardware. If the wiring changes, change values here.

- `LED_PIN` (int)
	- Onboard status LED used by the blink test task in [src/main.cpp](src/main.cpp).
	- Default is `LED_BUILTIN`.

- `global_PIN_MOTOR_FL`, `global_PIN_MOTOR_FR`, `global_PIN_MOTOR_BL`, `global_PIN_MOTOR_BR` (gpio_num_t)
	- DShot/ESC signal pins for the 4 motors (Front-Left, Front-Right, Back-Left, Back-Right).
	- Used in `motorCtrl.init(...)` in [src/main.cpp](src/main.cpp).

- `global_PIN_BATTERY_VOLTAGE_MONITOR`, `global_PIN_BATTERY_CURRENT_MONITOR` (gpio_num_t)
	- ADC pins for battery voltage and current sensing.
	- Passed to `batteryMonitor.init(...)` in [src/main.cpp](src/main.cpp).

- `global_PIN_IR_SENSOR_FL`, `global_PIN_IR_SENSOR_FR`, `global_PIN_IR_SENSOR_BM` (gpio_num_t)
	- ADC pins for the 3 IR line sensors (Front-Left, Front-Right, Back-Middle).
	- Sampled via `analogRead(...)` in `task_irSensors` in [src/main.cpp](src/main.cpp) (mapping/order: index0=BM, index1=FL, index2=FR).
	- Passed into the `IRSensors` constructor; `IRSensors::begin()` currently creates the internal sample queue used between the sampling and processing tasks.

- `global_PIN_I2C_SDA`, `global_PIN_I2C_SCL` (gpio_num_t)
	- Intended I2C pin definitions for sensors.
	- Note: the current IMU implementation starts I2C via `Wire.begin((int)SDA_PIN,(int)SCL_PIN)` if the macros `SDA_PIN`/`SCL_PIN` are defined, otherwise it calls `Wire.begin()` defaults (see [lib/imu/src/imu.cpp](lib/imu/src/imu.cpp)). These globals are currently not referenced directly.

### Battery related limits and variables 
These constants configure how ADC readings are converted into real battery voltage/current, plus optional low-voltage thresholds.

- `global_BatteryVoltage_VoltageDividerRatio` (float)
	- Divider ratio used to reconstruct battery voltage from the ADC pin voltage.
	- The battery monitor computes: $V_{batt} = V_{pin} / ratio$ (see [lib/battery_monitor/src/battery_monitor.cpp](lib/battery_monitor/src/battery_monitor.cpp)).
	- For a simple resistor divider, a typical ratio is $ratio = R_{bottom} / (R_{top}+R_{bottom})$.

- `global_BatteryCurrent_VoltageDividerRatio` (float)
	- Scaling factor applied to the measured current-sense pin voltage.
	- The battery monitor computes `sensor_mV = iPin_mV * ratio`.
	- Keep at `1.0` if you have no divider/scaler on the current sense line.

- `global_BatteryCurrent_SensorScaler_AmpsPerVolt` (float)
	- Betaflight-style current scale, **in units of [0.1 mV]/A** (despite the variable name).
	- The battery monitor converts it to mV/A via `mvPerAmp = scale * 0.1` and computes $I(A)=sensor_{mV}/(mV/A)$.
	- Example: `130` means 13 mV/A.

- `global_BatteryVoltageLow_WarningLow` (float, Volts)
	- Intended warning threshold for low battery.
	- Note: currently not enforced automatically; battery voltage is mainly sent as telemetry (see the battery task in [src/main.cpp](src/main.cpp)).

- `global_BatteryVoltageLow_MotorCutoffLow` (float, Volts)
	- Intended hard cutoff threshold where motors should stop.
	- Note: currently not enforced automatically; you can implement a failsafe in the motor task or battery task based on this.

### Motor control variables 
These variables define motor output scaling and direction conventions.

- `global_AllMotorsScalePercent` (float, %)
	- Global power cap applied inside `MotorCtrl` to reduce maximum throttle.
	- Used when `MotorCtrl motorCtrl(global_AllMotorsScalePercent)` is constructed in [src/main.cpp](src/main.cpp).

- `global_NegativeRpmScaleFactor` (float)
	- Reverse-direction compensation factor.
	- Applied only when a motor command is negative, to boost reverse output if reverse thrust is weaker (used in [lib/motor_ctrl/src/motor_ctrl.cpp](lib/motor_ctrl/src/motor_ctrl.cpp)).

- `global_MotorsReversedFL`, `global_MotorsReversedFR`, `global_MotorsReversedBL`, `global_MotorsReversedBR` (bool)
	- Per-motor direction flags.
	- Passed into `motorCtrl.init(...)` in [src/main.cpp](src/main.cpp) so that “positive thrust” matches the physical direction of the mounted propellers.
	- These are non-`const` globals, but in the current firmware they are effectively “startup configuration”.

### Web piloting / UI presets 
These values define what the web UI offers (presets) and how user input is interpreted.

- `global_WebLiftPresetPercent_Array[]` (float array, %)
	- List of lift presets shown/cycled in the web UI.
	- Used by the web UI generation logic in [lib/network_piloting/src/network_piloting.cpp](lib/network_piloting/src/network_piloting.cpp).

- `global_WebLiftPresetPercent_Array_len` (size_t)
	- Length of the lift preset array.
	- Used to bound preset iteration in [lib/network_piloting/src/network_piloting.cpp](lib/network_piloting/src/network_piloting.cpp).

- `global_WebLiftPresetPercent_Array_startIndex` (int)
	- Which preset becomes the first “non-zero” selection after startup (startup selection is always 0).
	- Used by preset cycling logic in [lib/network_piloting/src/network_piloting.cpp](lib/network_piloting/src/network_piloting.cpp).

- `global_WebThrustPresetPercent` (float, %)
	- Scales the maximum thrust command coming from the web UI.
	- Implemented in the thrust callback in [src/main.cpp](src/main.cpp): `scaled = thrustPercent * (preset/100)`.

### Wifi SSID, PW, IP Adressen 
These parameters configure the ESP32 SoftAP and the webserver port.

- `global_WifiApSsid` (char[])
	- SSID (network name) for the ESP32 Access Point.
	- Used in `wifiManager.startAccessPoint(...)` in [src/main.cpp](src/main.cpp).

- `global_WifiApPassword` (char[])
	- WPA2 password for the Access Point (must be at least 8 characters).
	- Used in `wifiManager.startAccessPoint(...)` in [src/main.cpp](src/main.cpp).

- `global_WebServerPort` (uint16_t)
	- TCP port for the embedded webserver (typically 80).
	- Used to construct the webserver inside `NetworkPiloting` (see [lib/network_piloting/src/network_piloting.cpp](lib/network_piloting/src/network_piloting.cpp)).

### Gyro/IMU/Complementary filter settings 
These values tune the IMU yaw estimation.

- `global_ComplementaryFilter_yawAlpha` (float, unitless)
	- Complementary filter weight for yaw: higher means “trust gyro more, correct slowly with magnetometer”.
	- Used when constructing the IMU and when configuring the filter in the IMU task in [src/main.cpp](src/main.cpp).
	- Typical range is 0…1.

### Control loop constants 
These define timing of the main control loop.

- `global_ControlLoopRate_Hz` (float, Hz)
	- Target frequency of the motor/control task loop.
	- Used to compute the periodic delay in `task_motorManagement` in [src/main.cpp](src/main.cpp).

### PID controller constants 
Yaw-rate control tuning parameters.

- `global_YawRatePid_Kp`, `global_YawRatePid_Ki`, `global_YawRatePid_Kd` (float)
	- Gains for the yaw-rate PID controller.
	- Used to initialize `yawRatePid` in `task_motorManagement` in [src/main.cpp](src/main.cpp).

- `global_YawRatePid_OutputLimit` (float)
	- Output clamp for the PID command (expected mixer units are roughly -100…100).

- `global_YawRatePid_IntegratorLimit` (float)
	- Integrator clamp (anti-windup) in the same units as the PID output.

- `global_MaxYawRateSetpoint_dps` (float, deg/s)
	- Maps full steering input (±100%) to a yaw-rate setpoint.
	- Used in [src/main.cpp](src/main.cpp) to compute the yaw-rate setpoint from the user steering slider.

- `global_YawCenterSensitivity` (float, deg/s)
	- “Center sensitivity” for the yaw command curve: sets how responsive yaw feels for *small* steering deflections.
	- Interpreted as the approximate slope around stick/slider center (0%).
	- Used in [src/main.cpp](src/main.cpp) to shape the mapping from steering percent to yaw-rate setpoint.

- `global_YawRateExpo` (float, unitless, 0…1)
	- “Expo” shaping for the yaw command curve: higher expo makes the response softer around center and ramps up more strongly towards full deflection.
	- Used together with `global_YawCenterSensitivity` and `global_MaxYawRateSetpoint_dps` to compute the yaw-rate setpoint from the user steering slider.
	- Used in [src/main.cpp](src/main.cpp) in the yaw setpoint mapping.
	- **Visualization tool:** there is a curve visualizer at `3_Software/rates.html` (open it in a browser) to inspect the response curve.

