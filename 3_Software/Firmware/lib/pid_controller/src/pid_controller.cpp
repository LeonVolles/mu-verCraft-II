#include "pid_controller.h"

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

void PIDController::init(float kp, float ki, float kd, float outputLimit, float integratorLimit)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;

    _outputLimit = fabsf(outputLimit);
    _integratorLimit = fabsf(integratorLimit);

    reset();
    _initialized = true;
}

void PIDController::reset()
{
    _integrator = 0.0f;
    _prevMeasurement = 0.0f;
    _havePrevMeasurement = false;
}

float PIDController::update(float setpoint, float measurement, float dt_s)
{
    if (!_initialized)
        return 0.0f;

    // Guard against bogus dt.
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.2f)
    {
        _havePrevMeasurement = false;
        return 0.0f;
    }

    const float error = setpoint - measurement;

    // Derivative on measurement to avoid derivative kick from setpoint steps.
    float dTerm = 0.0f;
    if (_havePrevMeasurement && _kd != 0.0f)
    {
        const float dMeas = (measurement - _prevMeasurement) / dt_s;
        dTerm = -_kd * dMeas;
    }
    _prevMeasurement = measurement;
    _havePrevMeasurement = true;

    const float pTerm = _kp * error;

    // Candidate integrator update.
    float newIntegrator = _integrator;
    if (_ki != 0.0f)
    {
        newIntegrator += _ki * error * dt_s;
        if (_integratorLimit > 0.0f)
        {
            newIntegrator = clampf(newIntegrator, -_integratorLimit, _integratorLimit);
        }
    }

    const float uUnsat = pTerm + newIntegrator + dTerm;

    float u = uUnsat;
    if (_outputLimit > 0.0f)
    {
        u = clampf(u, -_outputLimit, _outputLimit);
    }

    // Simple anti-windup: if saturated and error drives further into saturation, freeze integrator.
    const bool saturated = (_outputLimit > 0.0f) && (u != uUnsat);
    if (!saturated)
    {
        _integrator = newIntegrator;
    }
    else
    {
        // If output is saturated high and error is positive (wants more), don't integrate.
        // If output is saturated low  and error is negative (wants less), don't integrate.
        const bool pushingFurther = ((u > 0.0f) && (error > 0.0f)) || ((u < 0.0f) && (error < 0.0f));
        if (!pushingFurther)
        {
            _integrator = newIntegrator;
        }
    }

    return u;
}
