#include "imu.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

IMU::IMU()
    : _mpu(nullptr), _ready(false), _calibrated(false),
      _roll0_deg(0.0f), _pitch0_deg(0.0f), _g0(9.80665f),
      _ax_bias(0.0f), _ay_bias(0.0f) {}

IMU::~IMU()
{
    delete _mpu;
    _mpu = nullptr;
}

void IMU::init()
{
    if (_ready)
        return;

    // Initialize I2C if not already
    Wire.begin();

    _mpu = new Adafruit_MPU6050();
    if (!_mpu->begin())
    {
        Serial.println("MPU6050 not found");
        _ready = false;
        return;
    }

    // Basic config: ranges and filters suitable for general motion
    _mpu->setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu->setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu->setFilterBandwidth(MPU6050_BAND_21_HZ);

    _ready = true;
    _calibrated = false;
    _ax_bias = 0.0f;
    _ay_bias = 0.0f;
}

void IMU::getAccel_raw(float *ax, float *ay, float *az)
{
    if (ax)
        *ax = 0.0f;
    if (ay)
        *ay = 0.0f;
    if (az)
        *az = 0.0f;
    if (!_ready)
        return;

    sensors_event_t a, g, t;
    _mpu->getEvent(&a, &g, &t);
    if (ax)
        *ax = a.acceleration.x; // m/s^2
    if (ay)
        *ay = a.acceleration.y;
    if (az)
        *az = a.acceleration.z;
}

void IMU::getGyro_raw(float *gx, float *gy, float *gz)
{
    if (gx)
        *gx = 0.0f;
    if (gy)
        *gy = 0.0f;
    if (gz)
        *gz = 0.0f;
    if (!_ready)
        return;

    sensors_event_t a, g, t;
    _mpu->getEvent(&a, &g, &t);
    // Adafruit reports rad/s
    if (gx)
        *gx = g.gyro.x; // rad/s
    if (gy)
        *gy = g.gyro.y;
    if (gz)
        *gz = g.gyro.z;
}

void IMU::getAngles_raw(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    // Note: quick tilt from accelerometer only; yaw requires mag/gyro integration.
    float ax = 0, ay = 0, az = 0;
    getAccel_raw(&ax, &ay, &az);

    float roll = 0.0f;
    float pitch = 0.0f;

    // Protect against divide-by-zero
    const float eps = 1e-6f;
    float denom = sqrtf(ay * ay + az * az) + eps;
    roll = atan2f(ay, az) * 180.0f / (float)M_PI;
    pitch = atan2f(-ax, denom) * 180.0f / (float)M_PI;

    if (roll_deg)
        *roll_deg = roll;
    if (pitch_deg)
        *pitch_deg = pitch;
    if (yaw_deg)
        *yaw_deg = 0.0f; // not observable without magnetometer/fusion
}

void IMU::calibrateAccelReference(uint16_t samples, uint16_t sample_delay_ms)
{
    if (!_ready)
        return;

    // Average accelerometer while stationary
    double sumx = 0, sumy = 0, sumz = 0;
    for (uint16_t i = 0; i < samples; ++i)
    {
        float ax, ay, az;
        getAccel_raw(&ax, &ay, &az);
        sumx += ax;
        sumy += ay;
        sumz += az;
        delay(sample_delay_ms);
    }

    float ax0 = (float)(sumx / samples);
    float ay0 = (float)(sumy / samples);
    float az0 = (float)(sumz / samples);

    _g0 = sqrtf(ax0 * ax0 + ay0 * ay0 + az0 * az0);

    // Store initial tilt angles (deg)
    const float eps = 1e-6f;
    float roll0 = atan2f(ay0, az0) * 180.0f / (float)M_PI;
    float pitch0 = atan2f(-ax0, sqrtf(ay0 * ay0 + az0 * az0) + eps) * 180.0f / (float)M_PI;

    _roll0_deg = roll0;
    _pitch0_deg = pitch0;
    // Compute residual biases in level frame by rotating the averaged raw vector
    {
        const float r = _roll0_deg * (float)M_PI / 180.0f;
        const float p = _pitch0_deg * (float)M_PI / 180.0f;
        const float cr = cosf(r), sr = sinf(r);
        const float cp = cosf(p), sp = sinf(p);

        // Rx(-roll) then Ry(-pitch) applied to average raw vector
        float x1 = ax0;
        float y1 = cr * ay0 + sr * az0;
        float z1 = -sr * ay0 + cr * az0;

        float axc0 = cp * x1 - sp * z1;
        float ayc0 = y1;

        _ax_bias = axc0;
        _ay_bias = ayc0;
    }
    _calibrated = true;
}

static inline void rotateToLevel(float x, float y, float z, float roll_deg, float pitch_deg,
                                 float &xo, float &yo, float &zo)
{
    // Map sensor frame to level frame using inverse of calibration:
    // v_level = Ry(-pitch0) * Rx(-roll0) * v_sensor
    const float r = roll_deg * (float)M_PI / 180.0f;
    const float p = pitch_deg * (float)M_PI / 180.0f;
    const float cr = cosf(r), sr = sinf(r);
    const float cp = cosf(p), sp = sinf(p);

    // First Rx(-roll)
    float x1 = x;
    float y1 = cr * y + sr * z;  // y1 = c*y + s*z
    float z1 = -sr * y + cr * z; // z1 = -s*y + c*z

    // Then Ry(-pitch)
    xo = cp * x1 - sp * z1; // x2 = c*x1 - s*z1
    yo = y1;                // y unchanged for Ry
    zo = sp * x1 + cp * z1; // z2 = s*x1 + c*z1
}

void IMU::getAccel_corrected(float *ax, float *ay, float *az)
{
    float rx = 0, ry = 0, rz = 0;
    getAccel_raw(&rx, &ry, &rz);

    if (_calibrated)
    {
        float cx, cy, cz;
        rotateToLevel(rx, ry, rz, _roll0_deg, _pitch0_deg, cx, cy, cz);
        // Subtract small residual biases measured in calibration pose
        cx -= _ax_bias;
        cy -= _ay_bias;
        if (ax)
            *ax = cx;
        if (ay)
            *ay = cy;
        if (az)
            *az = cz;
    }
    else
    {
        if (ax)
            *ax = rx;
        if (ay)
            *ay = ry;
        if (az)
            *az = rz;
    }
}

void IMU::getAngles_corrected(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float ax, ay, az;
    getAccel_corrected(&ax, &ay, &az);

    const float eps = 1e-6f;
    float roll = atan2f(ay, az) * 180.0f / (float)M_PI;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az) + eps) * 180.0f / (float)M_PI;

    if (roll_deg)
        *roll_deg = roll;
    if (pitch_deg)
        *pitch_deg = pitch;
    if (yaw_deg)
        *yaw_deg = 0.0f;
}