#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// Forward declaration to avoid pulling the Adafruit header into every user
class Adafruit_MPU6050;

class IMU
{
private:
    Adafruit_MPU6050 *_mpu; // allocated in init()
    bool _ready;

public:
    IMU();
    ~IMU();

    // Initialize the sensor (Wire + MPU6050). Safe to call once.
    void init();
    bool isReady() const { return _ready; }

    // Raw sensor accessors (SI units):
    // - Acceleration in m/s^2
    // - Angular rates in rad/s
    void getAccel_raw(float *ax, float *ay, float *az);
    void getGyro_raw(float *gx, float *gy, float *gz);

    // Quick tilt estimate (deg) derived from accelerometer only.
    // Yaw is not observable without mag/gyro integration, returned as 0.
    void getAngles_raw(float *roll_deg, float *pitch_deg, float *yaw_deg);
};

#endif // IMU_H