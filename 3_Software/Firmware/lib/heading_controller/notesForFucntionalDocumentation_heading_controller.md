<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## Heading controller (outer loop for cascaded yaw control)

### Main idea
This module implements the **outer loop** of a cascaded yaw controller:

- **Outer loop (this module):** heading error (deg) → yaw-rate setpoint (deg/s)
- **Inner loop (existing):** yaw-rate setpoint (deg/s) → mixer `diffThrust` command (percent)

This allows commanding an **absolute heading** (relative to North as provided by the magnetometer-corrected complementary filter) instead of commanding yaw rate directly.

### Where it is used / how it is called
- Class/API: [lib/heading_controller/include/heading_controller.h](lib/heading_controller/include/heading_controller.h)
- Implementation: [lib/heading_controller/src/heading_controller.cpp](lib/heading_controller/src/heading_controller.cpp)
- Intended call site: the motor/control task (`task_motorManagement()`) in [src/main.cpp](src/main.cpp), when `autoMode`/heading-hold is enabled.

### Functionalities implemented
- **Angle wrap-around handling:** computes smallest signed heading error in $[-180, 180]$ degrees.
- **PID structure:** P + I + D terms.
- **Derivative on measurement:** uses measured yaw rate (deg/s) as heading derivative.
- **Output limiting:** clamps yaw-rate setpoint to $
\pm$`outputLimit_dps`.
- **Integrator limiting + anti-windup:** clamps I-term accumulator and freezes it when saturated and the error would push further.

### Configuration parameters
From project settings in [src/hovercraft_variables.h](src/hovercraft_variables.h):
- `global_HeadingPid_Kp`, `global_HeadingPid_Ki`, `global_HeadingPid_Kd`
- `global_HeadingPid_OutputLimit_dps`
- `global_HeadingPid_IntegratorLimit_dps`

### Notes / assumptions
- Headings are treated as degrees and normalized to $[0, 360)$.
- The controller output is a yaw-rate request (deg/s). The inner yaw-rate controller must still be active for actual actuation.
