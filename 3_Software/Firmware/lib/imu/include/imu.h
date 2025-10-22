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
    // Calibration state (tilt relative to gravity at rest)
    bool _calibrated;
    float _roll0_deg;  // initial roll (deg) from accel
    float _pitch0_deg; // initial pitch (deg) from accel
    float _g0;         // average gravity magnitude (m/s^2)
    // Residual biases in level frame (computed at calibration)
    float _ax_bias; // m/s^2
    float _ay_bias; // m/s^2

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

    // Calibrate tilt relative to gravity using averaged accelerometer samples.
    // Assumes the craft is stationary on a level reference surface during calibration.
    void calibrateAccelReference(uint16_t samples = 200, uint16_t sample_delay_ms = 5);
    bool isCalibrated() const { return _calibrated; }

    // Corrected accelerations rotated to a "level" frame where Z aligns with gravity
    // when at the calibration pose. If not calibrated, returns raw values.
    void getAccel_corrected(float *ax, float *ay, float *az);

    // Angles computed from corrected acceleration (deg). Yaw = 0.
    void getAngles_corrected(float *roll_deg, float *pitch_deg, float *yaw_deg);

    // Calibrated gravity magnitude (m/s^2). Defaults to ~9.80665 until calibrated.
    float getGravity_mps2() const { return _g0; }
};

#endif // IMU_H