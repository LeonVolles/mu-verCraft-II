<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## IMU (ADXL345 + ITG3205 + QMC5883 + BMP280) and yaw estimation

### Main idea
This module provides a single interface to the hovercraft’s IMU sensors (accel/gyro/mag/baro) and maintains a yaw estimate using a complementary filter (fast gyro integration + slow magnetometer correction).

### Where it is used / how it is called
- Class/API: [lib/imu/include/imu.h](lib/imu/include/imu.h) (implementation in [lib/imu/src/imu.cpp](lib/imu/src/imu.cpp)).
- Instantiated as a global object in [src/main.cpp](src/main.cpp) as `IMU imu(global_ComplementaryFilter_yawAlpha)`.
- Initialized in the RTOS task `task_imu()` in [src/main.cpp](src/main.cpp) via `imu.init()`, then continuously updated.
- The IMU task publishes the measured yaw rate into shared globals (`g_yawRateMeasured_dps`, `g_lastYawRateUpdate_us`) which are consumed by `task_motorManagement()` for yaw-rate control.

### Functionalities implemented
- **I2C + sensor bring-up:** starts I2C and probes/initializes each sensor; exposes presence flags (`hasAccel()`, `hasGyro()`, `hasMag()`, `hasBaro()`).
- **Raw sensor access in SI units:**
	- accelerometer in $m/s^2$ (`getAccel_raw()`)
	- gyro in $rad/s$ (`getGyro_raw()`)
	- magnetometer raw axes + heading in degrees (`getMag_raw()`)
	- barometer temperature + pressure (`getEnv()`)
- **Accel reference calibration (optional):** averages accel samples while the craft is still/level to define a “level frame” and biases; enables corrected accel/angles (`getAccel_corrected()`, `getAngles_corrected()`).
- **Yaw complementary filter:** integrates gyro Z-rate at high rate and corrects drift toward magnetometer heading when available (`updateYawComplementaryFrom()`), using a tunable alpha.
- **Yaw reset to magnetometer:** `resetYawToMag()` forces the filter to re-initialize yaw on the next update.

### Heading convention / mounting correction (important)
The magnetometer heading is **not** used “raw”. A hardcoded mounting correction is applied inside `IMU::getMag_raw()` to match the craft’s physical mounting and desired positive rotation direction.

- Raw magnetometer heading (after declination) is computed from `atan2(Y, X)` and normalized to $[0, 360)$.
- Then we apply the transform:
	- `heading_corrected = wrap360(180 - heading_raw)`

This single expression does two things:
- **180° offset** (board mounted reversed: world 0° corresponds to craft 180°)
- **Direction inversion** (so the heading increases in the desired “positive” rotation sense)

Because this mapping inverts heading direction, the yaw complementary filter must use a **matching gyro integration sign** so that gyro prediction and magnetometer correction agree. This is handled inside `IMU::updateYawComplementaryFrom()`.

### Methods (overview)
- `IMU(yawAlpha)` – constructs the sensor objects and sets yaw filter alpha.
- `init()` – starts I2C, probes sensors, sets ranges/modes, and resets yaw filter state.
- `has*()` / `isReady()` – capability/status flags.
- `getAccel_raw()`, `getGyro_raw()`, `getMag_raw()`, `getEnv()` – sensor readouts.
- `setDeclinationDeg()` – sets magnetometer declination correction.
- `getAngles_raw()` – roll/pitch from accel (yaw not observable here, returned as 0).
- `calibrateAccelReference()` / `isCalibrated()` – establish the reference tilt/gravity for corrected accel.
- `getAccel_corrected()`, `getAngles_corrected()` – accel rotated into the calibrated level frame.
- `updateYawComplementary()` / `updateYawComplementaryFrom(gz_rad_s, mag_heading_deg)` – yaw estimation update.
- `setYawFilterAlpha(alpha)`, `resetYawToMag()`, `getYaw_deg()`, `getYawGyro_deg()` – yaw filter configuration/output.

### Parameters used

#### Internal (class state)
- Sensor presence flags (`_accelOk`, `_gyroOk`, `_magOk`, `_bmpOk`) and `_ready`.
- Yaw filter state: `_yaw_deg`, `_yawGyro_deg`, `_yawAlpha`, `_lastYawUpdate_us`, init flags.
- Accel calibration: `_roll0_deg`, `_pitch0_deg`, `_ax_bias`, `_ay_bias`, `_g0`.
- Declination: `_declinationRad`.

#### External (project config)
Used in [src/main.cpp](src/main.cpp) / [src/hovercraft_variables.h](src/hovercraft_variables.h):
- `global_ComplementaryFilter_yawAlpha` – sets how strongly yaw trusts the gyro vs. magnetometer ($\alpha \in [0,1]$).

Note: the IMU module itself starts I2C using `SDA_PIN`/`SCL_PIN` only if those macros are defined at build time; otherwise it calls `Wire.begin()` with default pins.

