#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController
{
public:
    PIDController() = default;

    // Initialize PID gains and limits.
    // - outputLimit: clamps controller output to +/- outputLimit (in output units)
    // - integratorLimit: clamps internal I term to +/- integratorLimit (in output units)
    void init(float kp, float ki, float kd, float outputLimit, float integratorLimit);

    // Update PID controller and return output.
    // Units are user-defined but must be consistent:
    // - setpoint and measurement share the same units (e.g., deg/s)
    // - dt is in seconds
    float update(float setpoint, float measurement, float dt_s);

    // Reset I term and derivative memory.
    void reset();

    bool isInitialized() const { return _initialized; }

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _integrator = 0.0f;
    float _prevMeasurement = 0.0f;
    bool _havePrevMeasurement = false;

    float _outputLimit = 0.0f;
    float _integratorLimit = 0.0f;

    bool _initialized = false;
};

#endif // PID_CONTROLLER_H