<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->

## IR sensors (line detection)

### Main idea
This module turns three downward-facing IR sensor signals into **(1)** an angle $\alpha$ between the hovercraft and the line, and **(2)** a velocity component perpendicular to the line $v_{\perp}$. This can later be used for navigation and/or to correct long-term drift from integrated sensors (e.g. yaw).

### Where it is used / how it is called
- The class is `IRSensors` in [lib/ir_sensors/include/ir_sensors.h](lib/ir_sensors/include/ir_sensors.h) with the implementation in [lib/ir_sensors/src/ir_sensors.cpp](lib/ir_sensors/src/ir_sensors.cpp).
- A global instance is constructed in [src/main.cpp](src/main.cpp) using pins and geometry from [src/hovercraft_variables.h](src/hovercraft_variables.h).
- Runtime is split into **two RTOS tasks** in [src/main.cpp](src/main.cpp):
	- `task_irSensors`: samples the 3 sensors via ADC and enqueues samples.
	- `task_calcIrSensors`: drains the queue, runs line detection, then reads `alpha` and `v_perp` via getters.

### Functionalities implemented (global view)
- **High-rate sampling pipeline**: fast ADC sampling is decoupled from slower processing using a FreeRTOS queue (`enqueueSample` / `processQueue`).
- **Robust line event detection**: per-sensor rising-edge detection with a configurable threshold + hysteresis and an “armed/disarmed” latch to avoid repeated triggers.
- **Timeout handling**: if a line crossing is incomplete (only some sensors triggered) for too long, the partial state is reset.
- **Geometry-based estimation**: when all 3 sensors trigger for a crossing event, the code computes:
	- $\alpha$ (degrees): relative angle of the craft vs. the line.
	- $v_{\perp}$ (m/s): speed perpendicular to the line, with sign inferred from which sensors cross first.

### How the algorithm works (conceptual)
1. The sampling task reads 3 ADC values (BM/FL/FR) and timestamps each sample with `micros()`.
2. The processing task runs `processQueue(threshold, hysteresis)` which calls `detectLine(...)` for each sample.
3. `detectLine(...)` looks for a **rising crossing event** per sensor:
	 - below threshold → re-arm
	 - rising above (threshold + hysteresis) → trigger once and store timestamp
4. When all three sensors have triggered for the same crossing, the module computes $\alpha$ and $v_{\perp}$ using the configured sensor geometry.

### Methods overview
- `IRSensors(pinBM, pinFL, pinFR, a_m, b_m)`
	- Stores wiring + geometry. In this project the index mapping is **0=BM**, **1=FL**, **2=FR**.
- `begin()`
	- Creates the internal FreeRTOS sample queue used to decouple sampling from processing.
- `enqueueSample(values[3], t_us[3])`
	- Pushes one sample triplet into the queue (non-blocking).
- `processQueue(threshold, hysteresis)`
	- Drains the queue and calls `detectLine(...)` for each sample.
- `detectLine(values[3], t_us[3], threshold, hysteresis)`
	- Implements the actual line edge detection, timeout handling, and (when complete) $\alpha$ / $v_{\perp}$ computation.
- `getAlphaToLine()`, `getVelocityPerpToLine()`
	- Return the last computed results (may be `NAN` if no valid crossing happened yet).
- `hasNewMeasurement()`, `consumeNewMeasurement()`
	- Optional “new data” flag; can be used by callers to only react once per computed crossing.

### Parameters used (internal + extern)
From construction / internal state:
- Sensor pins (int): the three ADC pins (BM/FL/FR).
- Geometry (meters):
	- `_a`: side length from BM to FL/FR.
	- `_b`: base length between FL and FR.
- Internal trigger state: per-sensor `_armed[]`, `_prevValue[]`, and timestamps `_t1_us/_t2_us/_t3_us`.

From [src/hovercraft_variables.h](src/hovercraft_variables.h):
- `global_PIN_IR_SENSOR_BM`, `global_PIN_IR_SENSOR_FL`, `global_PIN_IR_SENSOR_FR` (pins).
- `global_IRSensorDistance_a_meters`, `global_IRSensorDistance_b_meters` (geometry).
- `global_IRSensor_Threshold` (ADC threshold) and `global_IRSensor_Hysteresis` (hysteresis band).
- `global_IRSensor_Timeout_us` (max allowed time for an incomplete crossing before reset).

Notes for maintainers:
- The current firmware uses **ADC sampling + queue**, not GPIO interrupts, for the IR sensors (see `task_irSensors` / `task_calcIrSensors` in [src/main.cpp](src/main.cpp)).
- Queue sizing (`SAMPLE_QUEUE_LEN`) is large in the header; if you run into RAM pressure, reducing it is a first place to look.
