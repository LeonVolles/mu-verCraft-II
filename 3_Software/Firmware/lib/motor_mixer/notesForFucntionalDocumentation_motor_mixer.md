<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## Motor mixer (lift + thrust + yaw differential)

### Main idea
This module converts high-level pilot/controller commands (lift, thrust, differential thrust) into four per-motor percent commands and forwards them to the motor controller.

### Where it is used / how it is called
- Class/API: [lib/motor_mixer/include/motor_mixer.h](lib/motor_mixer/include/motor_mixer.h) (implementation in [lib/motor_mixer/src/motor_mixer.cpp](lib/motor_mixer/src/motor_mixer.cpp)).
- Instantiated as a global object in [src/main.cpp](src/main.cpp) as `MotorMixer motorMixer(motorCtrl)` (it requires an existing `MotorCtrl`).
- Initialized in `setup()` via `motorMixer.init()`.
- Called periodically from `task_motorManagement()` in [src/main.cpp](src/main.cpp): it sets lift/thrust/diff commands (including PID output) via `setLift()`, `setThrust()`, and `setDiffThrust()`.

### Functionalities implemented
- **Lift mapping (front motors):** front-left and front-right always receive the same lift command in $[0,100]\%$.
- **Rear thrust mapping (rear motors):** back-left/back-right receive a base thrust in $[-100,100]\%$ plus an *absolute differential* term for yaw control.
- **Differential authority limiting:** `diffThrust` is mapped to a fixed maximum delta (currently 30% of output range) so yaw control cannot fully dominate rear thrust.
- **Deadband around zero:** rear motors are forced to 0 when magnitude is below 5% to avoid “buzzing” around the ESC dead zone.
- **Clamping/saturation:** all commands are constrained to valid percent ranges before sending to `MotorCtrl`.

### Methods (overview)
- `MotorMixer(MotorCtrl &motorCtrl)` – stores a reference to the motor controller used for output.
- `init()` – resets internal commands to 0 and sends 0% to all motors.
- `setLift(lift)` – updates lift command (0..100) and applies outputs.
- `setThrust(thrust)` – updates rear base thrust (-100..100) and applies outputs.
- `setDiffThrust(diffThrust)` – updates yaw differential command (-100..100) and applies outputs.
- `setLiftThrustDiff(lift, thrust, diffThrust)` – sets all three at once (avoids intermediate output states).
- `getLift() / getThrust() / getDiffThrust()` – returns last commanded values.

### Parameters used

#### Internal (class state)
- `m_lift`, `m_thrust`, `m_diffThrustBalance` – current command state (percent).
- `m_motorCtrl` – reference to the owned motor output layer.
- Mixer constants in `updateOutputs()`:
	- `diffAbsMaxPercent = 30` (max yaw delta applied to rear motors)
	- rear deadband of `5%`

#### External (project config / other modules)
- No direct dependency on [src/hovercraft_variables.h](src/hovercraft_variables.h) in this module.
- `setDiffThrust()` is typically fed by the yaw-rate PID in `task_motorManagement()` in [src/main.cpp](src/main.cpp) (PID constants live in [src/hovercraft_variables.h](src/hovercraft_variables.h)).
- Final safety gating and scaling are handled in `MotorCtrl` (see [lib/motor_ctrl/include/motor_ctrl.h](lib/motor_ctrl/include/motor_ctrl.h)).

