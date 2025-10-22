#include "imu.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

IMU::IMU()
    : _mpu(nullptr), _ready(false) {}

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