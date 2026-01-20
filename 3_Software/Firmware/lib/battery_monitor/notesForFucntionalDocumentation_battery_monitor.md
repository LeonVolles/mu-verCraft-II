<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## Battery monitor (ADC voltage + current + used capacity)

### Main idea
This module reads battery voltage and current from ADC pins, applies simple calibration (divider ratios + current scale), and integrates current over time to estimate used capacity in mAh.

### Where it is used / how it is called
- Class/API: [lib/battery_monitor/include/battery_monitor.h](lib/battery_monitor/include/battery_monitor.h) (implementation in [lib/battery_monitor/src/battery_monitor.cpp](lib/battery_monitor/src/battery_monitor.cpp)).
- Instantiated as a global object in [src/main.cpp](src/main.cpp) and initialized in `setup()` via `batteryMonitor.init(...)`.
- Updated periodically inside the RTOS task `task_batteryMonitor()` in [src/main.cpp](src/main.cpp).
- The task updates shared telemetry values (voltage/current/mAh) that are exposed to the UI via the `/debug` endpoint.

### Functionalities implemented
- **Voltage measurement (V):** reads an ADC pin in mV and scales it back to battery voltage using a divider ratio.
- **Current measurement (A):** reads an ADC pin in mV, applies an optional sensor divider ratio + optional zero-current offset, then converts sensor mV to amps using a Betaflight-style scale.
- **Used capacity integration (mAh):** integrates $I(t)$ over time using `millis()` delta time to estimate consumed mAh.
- **ESP32 ADC convenience:** on ESP32, uses `analogReadMilliVolts()` and configures ADC resolution + attenuation; other targets fall back to a generic 10-bit/3.3V conversion.
- **Low-voltage handling (system integration):** the RTOS battery task uses the measured battery voltage to drive a low-battery LED indicator and to trigger a motor cutoff when voltage stays below a configured threshold for multiple samples.

### Methods (overview)
- `BatteryMonitor()` – sets safe defaults (pins disabled, zeroed outputs).
- `init(voltageAdcPin, currentAdcPin, voltageDividerRatio, currentDividerRatio, betaflightScale_0p1mVPerAmp)` – configures pins and calibration constants; captures initial timestamp.
- `setCurrentOffset_mV(offset_mV)` – removes a fixed sensor offset (for sensors not centered at 0mV @ 0A).
- `update()` – reads ADCs, updates `voltage_V`, `current_A`, and accumulates `used_mAh` based on elapsed time.
- `getVoltage() / getCurrent() / getMAH()` – accessors for the latest computed values.
- `setMAH(mAh)` – allows resetting/overwriting the integrated capacity counter.

### Parameters used

#### Internal (class state)
- `_voltageAdcPin`, `_currentAdcPin` – ADC pins for voltage/current.
- `_voltageDividerRatio` – converts ADC pin voltage to battery voltage.
- `_currentDividerRatio` – converts ADC pin voltage to the sensor voltage (useful if there is an additional divider).
- `_betaflightScale_0p1mVPerAmp` – current conversion constant (see next section).
- `_currentOffset_mV` – optional zero-current offset.
- `_lastUpdateMs` – timestamp used to compute $\Delta t$ for mAh integration.

#### External (from hovercraft variables)
Configured in `setup()` in [src/main.cpp](src/main.cpp) using constants declared in [src/hovercraft_variables.h](src/hovercraft_variables.h):
- `global_PIN_BATTERY_VOLTAGE_MONITOR`, `global_PIN_BATTERY_CURRENT_MONITOR`
- `global_BatteryVoltage_VoltageDividerRatio`
- `global_BatteryCurrent_VoltageDividerRatio`
- `global_BatteryCurrent_SensorScaler_AmpsPerVolt`

Low-voltage thresholds used by the battery task in [src/main.cpp](src/main.cpp):
- `global_BatteryVoltageLow_WarningLow` – below this battery voltage the status LED is kept solid ON as a low-battery indicator.
- `global_BatteryVoltageLow_MotorCutoffLow` and `global_BatteryVoltageLow_MotorCutoffSamples` – motors are disabled if battery voltage is below the cutoff threshold for more than the configured number of consecutive samples.

Note: `BatteryMonitor::init()` expects the last parameter in **Betaflight units**: *scale in* $[0.1\text{ mV}]/A$ (example: `130` means $13\text{ mV}/A$). The variable name `global_BatteryCurrent_SensorScaler_AmpsPerVolt` is misleading compared to the code/comments; treat it as the Betaflight-style scale value used by this module.

