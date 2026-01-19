<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## PID controller (generic, rate-control oriented)

### Main idea
This module provides a lightweight PID controller that turns a setpoint/measurement error into a bounded output command, with practical safeguards (clamping, anti-windup, and derivative-on-measurement).

### Where it is used / how it is called
- Class/API: [lib/pid_controller/include/pid_controller.h](lib/pid_controller/include/pid_controller.h) (implementation in [lib/pid_controller/src/pid_controller.cpp](lib/pid_controller/src/pid_controller.cpp)).
- Instantiated locally in the motor/control task (`PIDController yawRatePid;`) inside `task_motorManagement()` in [src/main.cpp](src/main.cpp).
- Initialized once via `yawRatePid.init(...)` using constants from [src/hovercraft_variables.h](src/hovercraft_variables.h), then called each control loop with `yawRatePid.update(setpoint, measurement, dt_s)`.
- Called **once per iteration** of `task_motorManagement()`. The task is paced by `vTaskDelayUntil(...)` to a *target* period derived from `global_ControlLoopRate_Hz` (see [src/hovercraft_variables.h](src/hovercraft_variables.h)).
- The `dt_s` passed into `update()` is computed from `micros()` inside the task (so it reflects real loop timing, not just the requested RTOS delay) and is clamped to $[0.001, 0.05]$ seconds before being used.
- The current output is used as a yaw differential command and applied through `MotorMixer::setDiffThrust()` (see [lib/motor_mixer/include/motor_mixer.h](lib/motor_mixer/include/motor_mixer.h)).

### Functionalities implemented
- **P, I, D terms:** standard PID structure with configurable gains.
- **Derivative on measurement:** uses $-k_d\,\frac{d(\text{measurement})}{dt}$ to avoid derivative kick from setpoint steps.
- **Output limiting:** clamps controller output to $\pm$`outputLimit`.
- **Integrator limiting:** clamps the I-term accumulator to $\pm$`integratorLimit`.
- **Simple anti-windup:** if output saturates and the current error would push further into saturation, the integrator is frozen.
- **dt sanity guard:** returns 0 and clears derivative memory if $dt$ is invalid or too large (currently $dt > 0.2s$).

### Methods (overview)
- `init(kp, ki, kd, outputLimit, integratorLimit)` – sets gains/limits, resets state, marks controller initialized.
- `update(setpoint, measurement, dt_s)` – computes one PID step and returns the saturated output.
- `reset()` – clears integrator and derivative memory (useful when disarming or when measurements are stale).
- `isInitialized()` – indicates whether `init()` has been called.

### Parameters used

#### Internal (class state)
- `_kp`, `_ki`, `_kd` – gains for yaw.
- `_integrator` – accumulated I term.
- `_prevMeasurement`, `_havePrevMeasurement` – storage for derivative term.
- `_outputLimit`, `_integratorLimit` – clamps.

#### External (current usage in this project)
Configured in `task_motorManagement()` in [src/main.cpp](src/main.cpp) from [src/hovercraft_variables.h](src/hovercraft_variables.h):
- `global_YawRatePid_Kp`, `global_YawRatePid_Ki`, `global_YawRatePid_Kd`
- `global_YawRatePid_OutputLimit` (in mixer output units, typically `diffThrust` in -100..100)
- `global_YawRatePid_IntegratorLimit`
- `global_MaxYawRateSetpoint_dps` – used by the caller to convert the pilot “steering” percent command into a yaw-rate setpoint (deg/s) before feeding it into the PID.

Inputs/units are intentionally generic:
- `setpoint` and `measurement` must share units (here: yaw-rate in $deg/s$).
- `dt_s` is seconds.
- output unit is user-defined (here: `MotorMixer::setDiffThrust()` percent command).

