#pragma once

#include <Arduino.h>

// HeadingController: outer loop for cascaded yaw control.
// Input: target heading (deg, 0..360) and current heading (deg, 0..360).
// Output: desired yaw-rate setpoint (deg/s) to be fed into the inner yaw-rate PID.
class HeadingController
{
public:
    HeadingController() = default;

    // Initialize PID gains and limits.
    // - outputLimit_dps: clamps yaw-rate setpoint to +/- outputLimit_dps
    // - integratorLimit_dps: clamps the I-term accumulator to +/- integratorLimit_dps
    void init(float kp, float ki, float kd, float outputLimit_dps, float integratorLimit_dps);

    // One control step.
    // - targetHeading_deg/currentHeading_deg: in degrees (0..360 recommended)
    // - yawRateMeasured_dps: measured yaw rate (deg/s). Used as derivative-on-measurement.
    // - dt_s: seconds
    float update(float targetHeading_deg, float currentHeading_deg, float yawRateMeasured_dps, float dt_s);

    void reset();
    bool isInitialized() const { return _initialized; }

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _integrator = 0.0f;
    float _outputLimit = 0.0f;
    float _integratorLimit = 0.0f;

    bool _initialized = false;

    static float wrap360(float deg);
    static float angleDiffDeg(float target_deg, float current_deg);
    static float clampf(float v, float lo, float hi);
};
