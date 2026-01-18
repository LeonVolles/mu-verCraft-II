#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

#include <DFRobot_QMC5883.h>
#include <ITG3205.h>

class IMU
{
private:
    bool _ready = false;

    // Sensor availability
    bool _i2cStarted = false;
    bool _accelOk = false;
    bool _gyroOk = false;
    bool _magOk = false;
    bool _bmpOk = false;

    // Calibration state (tilt relative to gravity at rest)
    bool _calibrated = false;
    float _roll0_deg = 0.0f;  // initial roll (deg) from accel
    float _pitch0_deg = 0.0f; // initial pitch (deg) from accel
    float _g0 = 9.80665f;     // average gravity magnitude (m/s^2)
    // Residual biases in level frame (computed at calibration)
    float _ax_bias = 0.0f; // m/s^2
    float _ay_bias = 0.0f; // m/s^2

    // Magnetometer declination (radians). Default 0 = raw heading.
    float _declinationRad = 0.0f;

    // Yaw complementary filter (degrees)
    float _yaw_deg = 0.0f;
    float _yawGyro_deg = 0.0f; // gyro integrated only (no mag correction)
    bool _yawInitialized = false;
    bool _yawGyroInitialized = false;
    uint32_t _lastYawUpdate_us = 0;
    float _yawAlpha = 0.98f; // high = trust gyro more (overwritten from hovercraft_variables in ctor)

    // Sensor instances
    Adafruit_ADXL345_Unified _accel;
    Adafruit_BMP280 _bmp;
    DFRobot_QMC5883 _mag;
    ITG3205 _gyro;

    static void rotateToLevel(float roll_rad, float pitch_rad, float ax, float ay, float az,
                              float &outX, float &outY, float &outZ);

    static float wrap360(float deg);
    static float angleDiffDeg(float target_deg, float current_deg);

public:
    explicit IMU(float yawAlpha = 0.98f);
    ~IMU() = default;

    // Initialize I2C + sensors. Safe to call once.
    void init();
    bool isReady() const { return _ready; }

    // Sensor presence flags
    bool hasAccel() const { return _accelOk; }
    bool hasGyro() const { return _gyroOk; }
    bool hasMag() const { return _magOk; }
    bool hasBaro() const { return _bmpOk; }

    // Raw sensor accessors (SI units):
    // - Acceleration in m/s^2
    // - Angular rates in rad/s
    void getAccel_raw(float *ax, float *ay, float *az);
    void getGyro_raw(float *gx, float *gy, float *gz);

    // Optional sensors
    void getMag_raw(int16_t *mx, int16_t *my, int16_t *mz, float *heading_deg = nullptr);
    void getEnv(float *tempC, float *pressure_hPa);

    // Magnetometer declination helper
    void setDeclinationDeg(float declination_deg);

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

    // Calibrated gravity magnitude (m/s^2)
    float getGravity_mps2() const { return _g0; }

    // --- Yaw (Z-axis) complementary filter ---
    // Call this periodically (e.g. in your IMU task) to update the internal yaw estimate.
    // Uses gyro Z-rate integration and magnetometer heading as absolute reference.
    void updateYawComplementary();
    // Variant that uses already-read measurements:
    // - gz_rad_s: gyro Z rate in rad/s
    // - mag_heading_deg: magnetometer heading in degrees (use NAN if not available)
    void updateYawComplementaryFrom(float gz_rad_s, float mag_heading_deg);
    float getYaw_deg() const { return _yaw_deg; }
    float getYawGyro_deg() const { return _yawGyro_deg; }
    bool yawIsReady() const { return _yawInitialized; }
    bool yawGyroIsReady() const { return _yawGyroInitialized; }

    // alpha in [0..1]. Example: 0.98 = mostly gyro, slow mag correction.
    void setYawFilterAlpha(float alpha);

    // Forces yaw to current magnetometer heading on next update.
    void resetYawToMag();
};