#include "imu.h"
#include "../../../src/hovercraft_variables.h"
#include <math.h>

IMU::IMU(float yawAlpha)
    : _accel(12345), _mag(&Wire)
{
    setYawFilterAlpha(yawAlpha);
}

float IMU::wrap360(float deg)
{
    while (deg < 0.0f)
        deg += 360.0f;
    while (deg >= 360.0f)
        deg -= 360.0f;
    return deg;
}

// Returns shortest signed difference (target - current) in degrees, range [-180..180]
float IMU::angleDiffDeg(float target_deg, float current_deg)
{
    float diff = wrap360(target_deg) - wrap360(current_deg);
    if (diff > 180.0f)
        diff -= 360.0f;
    if (diff < -180.0f)
        diff += 360.0f;
    return diff;
}

void IMU::rotateToLevel(float roll_rad, float pitch_rad, float ax, float ay, float az,
                        float &outX, float &outY, float &outZ)
{
    // Apply inverse of the calibration pose: rotate by -roll and -pitch.
    const float cr = cosf(-roll_rad);
    const float sr = sinf(-roll_rad);
    const float cp = cosf(-pitch_rad);
    const float sp = sinf(-pitch_rad);

    // Rotate around X (roll)
    const float x1 = ax;
    const float y1 = cr * ay - sr * az;
    const float z1 = sr * ay + cr * az;

    // Rotate around Y (pitch)
    outX = cp * x1 + sp * z1;
    outY = y1;
    outZ = -sp * x1 + cp * z1;
}

void IMU::init()
{
    if (!_i2cStarted)
    {
        // Start I2C with project-defined pins if present.
#if defined(SDA_PIN) && defined(SCL_PIN)
        Wire.begin((int)SDA_PIN, (int)SCL_PIN);
#else
        Wire.begin();
#endif
        Wire.setClock(400000);
        _i2cStarted = true;
    }

    // ADXL345
    _accelOk = _accel.begin();
    if (_accelOk)
    {
        _accel.setRange(ADXL345_RANGE_16_G);
    }

    // BMP280 (0x76 or 0x77)
    _bmpOk = (_bmp.begin(0x77) || _bmp.begin(0x76));

    // QMC/HMC/VCM5883L
    _magOk = false;
    for (int attempt = 0; attempt < 5; attempt++)
    {
        if (_mag.begin())
        {
            _magOk = true;
            break;
        }
        delay(50);
    }
    if (_magOk)
    {
        // Keep 0.0f for raw heading unless you set it.
        _mag.setDeclinationAngle(_declinationRad);
    }

    // ITG3205 (0x68/0x69)
    _gyroOk = _gyro.probe(Wire);
    if (_gyroOk)
    {
        if (!_gyro.initialize())
        {
            _gyroOk = false;
        }
        else
        {
            // Calibrate bias quickly (keep craft still); no logging from inside lib.
            _gyro.calibrate(200, nullptr);
        }
    }

    // Consider IMU "ready" if at least accel is present.
    _ready = _accelOk;

    // Reset yaw filter state on init
    _yawInitialized = false;
    _yawGyroInitialized = false;
    _lastYawUpdate_us = micros();
}

void IMU::setDeclinationDeg(float declination_deg)
{
    _declinationRad = declination_deg * PI / 180.0f;
    if (_magOk)
    {
        _mag.setDeclinationAngle(_declinationRad);
    }
}

void IMU::setYawFilterAlpha(float alpha)
{
    if (!isfinite(alpha))
        return;
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;
    _yawAlpha = alpha;
}

void IMU::resetYawToMag()
{
    _yawInitialized = false;
    _yawGyroInitialized = false;
}

void IMU::updateYawComplementary()
{
    // Read sensors needed for yaw, then forward to the measurement-based update.
    float gz_rad_s = NAN;
    if (_gyroOk)
    {
        float gx, gy, gz;
        getGyro_raw(&gx, &gy, &gz);
        gz_rad_s = gz;
    }

    float mag_heading_deg = NAN;
    if (_magOk)
    {
        getMag_raw(nullptr, nullptr, nullptr, &mag_heading_deg);
    }

    updateYawComplementaryFrom(gz_rad_s, mag_heading_deg);
}

void IMU::updateYawComplementaryFrom(float gz_rad_s, float mag_heading_deg)
{
    const uint32_t now_us = micros();
    float dt = 0.0f;
    if (_lastYawUpdate_us != 0)
    {
        const uint32_t delta_us = now_us - _lastYawUpdate_us;
        dt = (float)delta_us / 1e6f;
    }
    _lastYawUpdate_us = now_us;

    const bool haveGyro = isfinite(gz_rad_s) && dt > 0.0f && dt < 1.0f;
    const bool haveMag = isfinite(mag_heading_deg);

    if (!_yawInitialized)
    {
        if (haveMag)
        {
            _yaw_deg = wrap360(mag_heading_deg);
            _yawInitialized = true;
            // Initialize gyro-only yaw to the same reference so both are comparable.
            _yawGyro_deg = _yaw_deg;
            _yawGyroInitialized = true;
            return;
        }

        // If no magnetometer available yet, start at 0 and integrate gyro when possible.
        _yaw_deg = wrap360(_yaw_deg);
        _yawInitialized = haveGyro;
        _yawGyro_deg = wrap360(_yawGyro_deg);
        _yawGyroInitialized = haveGyro;
        return;
    }

    if (!_yawGyroInitialized)
    {
        if (haveMag)
        {
            _yawGyro_deg = wrap360(mag_heading_deg);
            _yawGyroInitialized = true;
        }
        else if (haveGyro)
        {
            _yawGyroInitialized = true;
        }
    }

    // 1) Predict with gyro integration
    float yaw_pred = _yaw_deg;
    if (haveGyro)
    {
        const float gz_deg_s = gz_rad_s * 180.0f / PI;
        // Sign convention: keep gyro yaw integration consistent with magnetometer heading.
        // With the current board mounting, we apply an inverted heading convention, so gyro integration
        // must use the corresponding opposite sign compared to the raw sensor.
        yaw_pred = wrap360(yaw_pred + gz_deg_s * dt);
        // Gyro-only yaw integrates the same gyro rate but never gets mag correction.
        _yawGyro_deg = wrap360(_yawGyro_deg + gz_deg_s * dt);
    }

    // 2) Correct with magnetometer (complementary)
    if (haveMag)
    {
        const float err = angleDiffDeg(mag_heading_deg, yaw_pred);
        // Equivalent to: yaw = alpha*yaw_pred + (1-alpha)*mag, but done via shortest-angle correction.
        _yaw_deg = wrap360(yaw_pred + (1.0f - _yawAlpha) * err);
    }
    else
    {
        _yaw_deg = yaw_pred;
    }
}

void IMU::getAccel_raw(float *ax, float *ay, float *az)
{
    if (ax)
        *ax = NAN;
    if (ay)
        *ay = NAN;
    if (az)
        *az = NAN;

    if (!_accelOk)
        return;

    sensors_event_t event;
    _accel.getEvent(&event);

    if (ax)
        *ax = event.acceleration.x;
    if (ay)
        *ay = event.acceleration.y;
    if (az)
        *az = event.acceleration.z;
}

void IMU::getGyro_raw(float *gx, float *gy, float *gz)
{
    if (gx)
        *gx = NAN;
    if (gy)
        *gy = NAN;
    if (gz)
        *gz = NAN;

    if (!_gyroOk)
        return;

    float gxDps = NAN, gyDps = NAN, gzDps = NAN, tempC = NAN;
    if (!_gyro.read(gxDps, gyDps, gzDps, tempC))
        return;

    const float deg2rad = (PI / 180.0f);
    if (gx)
        *gx = gxDps * deg2rad;
    if (gy)
        *gy = gyDps * deg2rad;
    if (gz)
        *gz = gzDps * deg2rad;
}

void IMU::getMag_raw(int16_t *mx, int16_t *my, int16_t *mz, float *heading_deg)
{
    // return NaN if not available
    if (mx)
        *mx = 0;
    if (my)
        *my = 0;
    if (mz)
        *mz = 0;
    if (heading_deg)
        *heading_deg = NAN;

    if (!_magOk)
        return;

    // Read raw magnetometer values
    sVector_t v = _mag.readRaw();

    // --- ADDED FOR DEBUGGING ---
    // Print raw X, Y, Z to Serial so you can find Min/Max for calibration
    // Serial.printf("%0.2f,%0.2f,%0.2f\n", (float)v.XAxis, (float)v.YAxis, (float)v.ZAxis);

    // Apply hard-iron offsets (raw units)
    v.XAxis = (int16_t)(v.XAxis - global_MagOffsetX);
    v.YAxis = (int16_t)(v.YAxis - global_MagOffsetY);
    v.ZAxis = (int16_t)(v.ZAxis - global_MagOffsetZ);

    // Print corrected values for verification
    // Serial.printf("%0.2f,%0.2f,%0.2f\n", (float)v.XAxis, (float)v.YAxis, (float)v.ZAxis);

    if (mx)
        *mx = v.XAxis;
    if (my)
        *my = v.YAxis;
    if (mz)
        *mz = v.ZAxis;

    if (heading_deg)
    {
        float heading = atan2f((float)v.YAxis, (float)v.XAxis);
        heading += _declinationRad;
        if (heading < 0)
            heading += 2.0f * PI;
        if (heading > 2.0f * PI)
            heading -= 2.0f * PI;
        // Raw magnetometer heading in degrees, normalized to [0..360).
        float headingDeg = heading * 180.0f / PI;

        // Hardcoded mounting correction:
        // - Board is mounted reversed (0° world corresponds to 180° craft)
        // - Heading direction is inverted (values were decreasing when they should increase)
        // Combined transform: heading' = wrap360(180 - heading)
        headingDeg = wrap360(180.0f - headingDeg);

        *heading_deg = headingDeg;
    }
}

void IMU::getEnv(float *tempC, float *pressure_hPa)
{
    if (tempC)
        *tempC = NAN;
    if (pressure_hPa)
        *pressure_hPa = NAN;

    if (!_bmpOk)
        return;

    if (tempC)
        *tempC = _bmp.readTemperature();
    if (pressure_hPa)
        *pressure_hPa = _bmp.readPressure() / 100.0f;
}

void IMU::getAngles_raw(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (roll_deg)
        *roll_deg = NAN;
    if (pitch_deg)
        *pitch_deg = NAN;
    if (yaw_deg)
        *yaw_deg = 0.0f;

    float ax, ay, az;
    getAccel_raw(&ax, &ay, &az);
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az))
        return;

    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    if (roll_deg)
        *roll_deg = roll * 180.0f / PI;
    if (pitch_deg)
        *pitch_deg = pitch * 180.0f / PI;
}

void IMU::calibrateAccelReference(uint16_t samples, uint16_t sample_delay_ms)
{
    if (!_accelOk)
        return;

    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    uint16_t got = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        float ax, ay, az;
        getAccel_raw(&ax, &ay, &az);
        if (isfinite(ax) && isfinite(ay) && isfinite(az))
        {
            sumX += ax;
            sumY += ay;
            sumZ += az;
            got++;
        }
        delay(sample_delay_ms);
    }

    if (got == 0)
        return;

    const float axAvg = (float)(sumX / (double)got);
    const float ayAvg = (float)(sumY / (double)got);
    const float azAvg = (float)(sumZ / (double)got);

    const float roll0 = atan2f(ayAvg, azAvg);
    const float pitch0 = atan2f(-axAvg, sqrtf(ayAvg * ayAvg + azAvg * azAvg));

    _roll0_deg = roll0 * 180.0f / PI;
    _pitch0_deg = pitch0 * 180.0f / PI;

    float lx, ly, lz;
    rotateToLevel(roll0, pitch0, axAvg, ayAvg, azAvg, lx, ly, lz);

    _ax_bias = lx;
    _ay_bias = ly;
    _g0 = sqrtf(lx * lx + ly * ly + lz * lz);

    _calibrated = true;
}

void IMU::getAccel_corrected(float *ax, float *ay, float *az)
{
    float rawX, rawY, rawZ;
    getAccel_raw(&rawX, &rawY, &rawZ);

    if (!isfinite(rawX) || !isfinite(rawY) || !isfinite(rawZ))
    {
        if (ax)
            *ax = NAN;
        if (ay)
            *ay = NAN;
        if (az)
            *az = NAN;
        return;
    }

    if (!_calibrated)
    {
        if (ax)
            *ax = rawX;
        if (ay)
            *ay = rawY;
        if (az)
            *az = rawZ;
        return;
    }

    const float roll0 = _roll0_deg * PI / 180.0f;
    const float pitch0 = _pitch0_deg * PI / 180.0f;

    float lx, ly, lz;
    rotateToLevel(roll0, pitch0, rawX, rawY, rawZ, lx, ly, lz);

    if (ax)
        *ax = lx - _ax_bias;
    if (ay)
        *ay = ly - _ay_bias;
    if (az)
        *az = lz;
}

void IMU::getAngles_corrected(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (roll_deg)
        *roll_deg = NAN;
    if (pitch_deg)
        *pitch_deg = NAN;
    if (yaw_deg)
        *yaw_deg = 0.0f;

    float ax, ay, az;
    getAccel_corrected(&ax, &ay, &az);
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az))
        return;

    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    if (roll_deg)
        *roll_deg = roll * 180.0f / PI;
    if (pitch_deg)
        *pitch_deg = pitch * 180.0f / PI;
}
