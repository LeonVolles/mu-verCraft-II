#include "heading_controller.h"

float HeadingController::clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

float HeadingController::wrap360(float deg)
{
    if (!isfinite(deg))
        return 0.0f;
    while (deg >= 360.0f)
        deg -= 360.0f;
    while (deg < 0.0f)
        deg += 360.0f;
    return deg;
}

float HeadingController::angleDiffDeg(float target_deg, float current_deg)
{
    // Smallest signed difference (target-current) in [-180, 180].
    float t = wrap360(target_deg);
    float c = wrap360(current_deg);
    float d = t - c;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    return d;
}

void HeadingController::init(float kp, float ki, float kd, float outputLimit_dps, float integratorLimit_dps)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;

    _outputLimit = fabsf(outputLimit_dps);
    _integratorLimit = fabsf(integratorLimit_dps);

    reset();
    _initialized = true;
}

void HeadingController::reset()
{
    _integrator = 0.0f;
}

float HeadingController::update(float targetHeading_deg, float currentHeading_deg, float yawRateMeasured_dps, float dt_s)
{
    if (!_initialized)
        return 0.0f;

    // Guard against bogus dt.
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.2f)
        return 0.0f;

    const float error_deg = angleDiffDeg(targetHeading_deg, currentHeading_deg);

    const float pTerm = _kp * error_deg;

    float newIntegrator = _integrator;
    if (_ki != 0.0f)
    {
        newIntegrator += _ki * error_deg * dt_s;
        if (_integratorLimit > 0.0f)
        {
            newIntegrator = clampf(newIntegrator, -_integratorLimit, _integratorLimit);
        }
    }

    // Derivative on measurement: heading derivative is yaw rate.
    float dTerm = 0.0f;
    if (_kd != 0.0f && isfinite(yawRateMeasured_dps))
    {
        dTerm = -_kd * yawRateMeasured_dps;
    }

    const float uUnsat = pTerm + newIntegrator + dTerm;
    float u = uUnsat;
    if (_outputLimit > 0.0f)
        u = clampf(u, -_outputLimit, _outputLimit);

    // Simple anti-windup: if saturated and error drives further into saturation, freeze integrator.
    const bool saturated = (_outputLimit > 0.0f) && (u != uUnsat);
    if (!saturated)
    {
        _integrator = newIntegrator;
    }
    else
    {
        const bool pushingFurther = ((u > 0.0f) && (error_deg > 0.0f)) || ((u < 0.0f) && (error_deg < 0.0f));
        if (!pushingFurther)
        {
            _integrator = newIntegrator;
        }
    }

    return u;
}
