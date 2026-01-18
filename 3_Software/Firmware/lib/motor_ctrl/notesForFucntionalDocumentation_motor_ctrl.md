<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## Motor control (DShot ESC output)

### Main idea
This module owns the four ESC outputs and converts high-level motor commands (percent per motor) into DShot3D throttle values, while enforcing safety rules (arm/disarm, emergency off) and some small helpers for real hardware behavior.DShot is used, because it is more save than a PWM. Look in the Setup->ESC settings chapter, to find out how the ESC is configured.

### Where it is used / how it is called
- Class/API: [lib/motor_ctrl/include/motor_ctrl.h](lib/motor_ctrl/include/motor_ctrl.h) (implementation in [lib/motor_ctrl/src/motor_ctrl.cpp](lib/motor_ctrl/src/motor_ctrl.cpp)).
- Instantiated as a global object in [src/main.cpp](src/main.cpp) as `MotorCtrl motorCtrl(global_AllMotorsScalePercent)`.
- Initialized in `setup()` via `motorCtrl.init(...)` using pin + direction flags from [src/hovercraft_variables.h](src/hovercraft_variables.h).
- During runtime, `task_motorManagement()` in [src/main.cpp](src/main.cpp) controls the safety gate with `motorCtrl.setMotorsEnabled(...)` and applies lift cut via `motorCtrl.applyLiftOff()`.
- The mixer [lib/motor_mixer/include/motor_mixer.h](lib/motor_mixer/include/motor_mixer.h) holds a reference to the `MotorCtrl` instance and is the typical caller of the per-motor percent setters.

### Functionalities implemented
- **DShot3D output mapping:** converts percent commands $[-100,100]$ into DShot3D throttle commands $[-999,999]$.
- **Global power scaling:** applies `pGeneralMotorPowerScalerPercent` so the project can cap motor power globally (useful to avoid overloading motors/ESC/battery).
- **Emergency safety gate:** `setMotorsEnabled(false)` forces **all** motors to 0 immediately and overrides any mixer/setpoint logic.
- **Lift-only cut:** `applyLiftOff()` stops the front motors (lift) without touching the rear motors (thrust).
- **Startup/arming behavior:** `init()` sends 0-throttle for a long sequence to satisfy typical ESC arming requirements.
- **Kick-start helper:** when commanding from standstill to a “large” throttle, sends a brief small kick (≈20ms) before switching to the requested throttle (helps with motors that don’t start reliably from very low commands).
- **Direction handling:** supports per-motor wiring reversal flags, and applies a configurable reverse-direction compensation factor for negative commands.

### Methods (overview)
- `MotorCtrl(generalMotorPowerScalerPercent)` – sets internal state to 0 and starts **disabled** (must be armed explicitly).
- `init(fl_pin, fr_pin, bl_pin, br_pin, reversedFL, reversedFR, reversedBL, reversedBR)` – installs + initializes 4 DShot ESCs and performs the 0-throttle arming sequence.
- `setMotorsEnabled(enabled)` / `motorsEnabled()` / `EmergencyOff()` – global arming gate / emergency stop.
- `applyLiftOff()` – immediately stops only the lift motors (front left/right).
- `setFrontLeftPercent() / setFrontRightPercent() / setBackLeftPercent() / setBackRightPercent()` – per-motor commands in percent.
- `setAllPercent(fl, fr, bl, br)` – convenience wrapper for the 4 setters.
- `getFrontLeftPercent()` / ... – returns the last commanded percentages.
- `tempForDebug_*` – direct throttle debug helpers (bypass percent mapping, still respects the safety gate).

### Parameters used

#### Internal (class state)
- `pPercentFL/FR/BL/BR` – last requested percent commands.
- `pGeneralMotorPowerScalerPercent` – global percent scaling applied to all motors.
- `pMotorsEnabled` – safety gate; when false, outputs are forced to 0.
- `pSetReversed*` – per-motor reversal flags applied in software.
- `KickState` per motor – tracks the kick-start phase (last throttle, active flag, start time, pending desired command).

#### External (project config)
Used from [src/hovercraft_variables.h](src/hovercraft_variables.h) / [src/hovercraft_variables.cpp](src/hovercraft_variables.cpp):
- `global_AllMotorsScalePercent` – passed into the `MotorCtrl` constructor in [src/main.cpp](src/main.cpp).
- `global_PIN_MOTOR_FL/FR/BL/BR` – passed to `motorCtrl.init(...)`.
- `global_MotorsReversedFL/FR/BL/BR` – passed to `motorCtrl.init(...)` to invert commands per motor.
- `global_NegativeRpmScaleFactor` – multiplier applied to **negative** (reverse) motor commands before optional motor reversal. This is to combat insymetrical thrust due to the insymetrical propeller design. 

