#include "ir_sensors.h"
#include <math.h>

// Global instance pointer for ISR trampolines.
IRSensors *IRSensors::_instance = nullptr;

// Degree/radian conversion helpers.
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * (PI / 180.0f))
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * (180.0f / PI))
#endif

IRSensors::IRSensors(int pin1, int pin2, int pin3, float sensorDistance_a, float sensorDistance_b)
    : _pin1(pin1),
      _pin2(pin2),
      _pin3(pin3),
      _a(sensorDistance_a),
      _b(sensorDistance_b),
      _lastAlphaDeg(NAN),
      _lastVelPerp(NAN),
      _hasNewMeasurement(false),
      _t1_us(0),
      _t2_us(0),
      _t3_us(0),
      _t1_valid(false),
      _t2_valid(false),
      _t3_valid(false),
      _crossingStart_us(0)
{
}

// Initialize pins and interrupts.
void IRSensors::begin()
{
    _instance = this;

    pinMode(_pin1, INPUT_PULLUP);
    pinMode(_pin2, INPUT_PULLUP);
    pinMode(_pin3, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(_pin1), isrSensor1Trampoline, FALLING);
    attachInterrupt(digitalPinToInterrupt(_pin2), isrSensor2Trampoline, FALLING);
    attachInterrupt(digitalPinToInterrupt(_pin3), isrSensor3Trampoline, FALLING);
}

float IRSensors::getAlphaToLine() const
{
    return _lastAlphaDeg;
}

float IRSensors::getVelocityPerpToLine() const
{
    return _lastVelPerp;
}

bool IRSensors::hasNewMeasurement() const
{
    return _hasNewMeasurement;
}

void IRSensors::consumeNewMeasurement()
{
    _hasNewMeasurement = false;
}

// ---- Static helpers --------------------------------------------------

// Clamp helper: limits a value to the interval [-1, 1].
float IRSensors::clamp1(float v)
{
    if (v < -1.0f)
        return -1.0f;
    if (v > 1.0f)
        return 1.0f;
    return v;
}

// Normalizes an angle to the interval [0, 360).
float IRSensors::normalizeDeg(float a)
{
    while (a < 0.0f)
        a += 360.0f;
    while (a >= 360.0f)
        a -= 360.0f;
    return a;
}

// Computes alpha (in degrees) from one complete set of normalized dt values.
// The three equations are:
//   dt12_x = cos(alpha - 30°)
//   dt13_x = cos(alpha + 30°)
//   dt23_x = -sin(alpha)
// From each equation we obtain up to two possible alpha values, so we end up
// with several candidate angles stored in the array cand[].
// "cand" stands for "candidate angle".
float IRSensors::computeAlphaFromDt(float dt12, float dt13, float dt23)
{
    // Ensure numeric stability in inverse trig functions.
    dt12 = clamp1(dt12);
    dt13 = clamp1(dt13);
    dt23 = clamp1(dt23);

    // Array for candidate angles (in degrees).
    // cand[i] is the i-th candidate angle derived from inverse trig.
    float cand[6];
    int nCand = 0;

    // 1) Candidates from dt12_x = cos(alpha - 30°).
    {
        float c = dt12;
        if (fabsf(c) <= 1.0f)
        {
            float acosVal = RAD2DEG(acosf(c)); // acosf returns radians. [web:43]
            float a1 = 30.0f + acosVal;
            float a2 = 30.0f - acosVal;
            cand[nCand++] = normalizeDeg(a1);
            cand[nCand++] = normalizeDeg(a2);
        }
    }

    // 2) Candidates from dt13_x = cos(alpha + 30°).
    {
        float c = dt13;
        if (fabsf(c) <= 1.0f)
        {
            float acosVal = RAD2DEG(acosf(c));
            float a3 = -30.0f + acosVal;
            float a4 = -30.0f - acosVal;
            cand[nCand++] = normalizeDeg(a3);
            cand[nCand++] = normalizeDeg(a4);
        }
    }

    // 3) Candidates from dt23_x = -sin(alpha).
    {
        float s = -dt23;
        if (fabsf(s) <= 1.0f)
        {
            float asinVal = RAD2DEG(asinf(s)); // asinf returns radians. [web:43]
            float b1 = asinVal;                // first solution
            float b2 = 180.0f - asinVal;       // second solution
            cand[nCand++] = normalizeDeg(b1);
            cand[nCand++] = normalizeDeg(b2);
        }
    }

    // If no candidates are available, return NaN as an error indication.
    if (nCand == 0)
    {
        return NAN;
    }

    // 4) Select the best candidate using a least-squares error metric.
    // For each candidate angle we recompute expected dt*_x values and
    // accumulate the squared differences to the measured values.
    float bestAlpha = cand[0]; // best candidate angle found so far
    float bestErr = 1e30f;     // minimal error found so far

    for (int i = 0; i < nCand; ++i)
    {
        float aDeg = cand[i];
        float aRad = DEG2RAD(aDeg);

        // Reconstructed values from this candidate angle.
        float dt12_est = cosf(aRad - DEG2RAD(30.0f));
        float dt13_est = cosf(aRad + DEG2RAD(30.0f));
        float dt23_est = -sinf(aRad);

        // Squared errors for each equation.
        float e12 = dt12_est - dt12;
        float e13 = dt13_est - dt13;
        float e23 = dt23_est - dt23;

        float err = e12 * e12 + e13 * e13 + e23 * e23;

        // Keep the candidate with minimal total error.
        if (err < bestErr)
        {
            bestErr = err;
            bestAlpha = aDeg;
        }
    }

    return normalizeDeg(bestAlpha);
}

// Computes the hovercraft velocity component perpendicular to the line (v_perp).
// Inputs:
//   alphaDeg : orientation angle alpha in degrees (result of computeAlphaFromDt()).
//   a        : distance between two neighboring sensors (in meters).
//   dt12     : time difference between sensor 1 and 2 crossings (in seconds).
//   dt13     : time difference between sensor 1 and 3 crossings (in seconds).
//   dt23     : time difference between sensor 2 and 3 crossings (in seconds).
//
// Assumptions:
// - The three sensors are mounted on a circle, forming an equilateral triangle
//   with side length a around the hovercraft center.
// - The hovercraft does not rotate during the line crossing (yaw is constant).
// - alphaDeg remains constant over the short crossing event.
//
// Method (conceptual):
// For each sensor pair i-j, a projected distance along the motion direction is
// computed from the geometry (a and alphaDeg). Dividing this projected distance
// by the measured time difference dt_ij yields a velocity estimate. The function
// averages all valid pairwise estimates to get a robust v_perp.
float IRSensors::computeVelocityPerpToLine(float alphaDeg,
                                           float a,
                                           float dt12,
                                           float dt13,
                                           float dt23)
{
    // Convert alpha to radians for trig.
    float alphaRad = DEG2RAD(alphaDeg);

    // Small epsilon to avoid division by zero.
    float epsilon = 1e-6f;

    // Projected distances for each sensor pair, consistent with dt*_x definitions.
    float d12_proj = a * cosf(alphaRad - DEG2RAD(30.0f));
    float d13_proj = a * cosf(alphaRad + DEG2RAD(30.0f));
    float d23_proj = a * sinf(alphaRad); // sign encodes direction along the line

    // Collect individual speed estimates (absolute value along motion direction).
    float vSum = 0.0f;
    int vCount = 0;

    if (fabsf(dt12) > epsilon)
    {
        float v12 = fabsf(d12_proj / dt12);
        vSum += v12;
        vCount++;
    }

    if (fabsf(dt13) > epsilon)
    {
        float v13 = fabsf(d13_proj / dt13);
        vSum += v13;
        vCount++;
    }

    if (fabsf(dt23) > epsilon)
    {
        float v23 = fabsf(d23_proj / dt23);
        vSum += v23;
        vCount++;
    }

    if (vCount == 0)
    {
        // No valid time differences, return NaN.
        return NAN;
    }

    // Average of all valid pair-based estimates.
    float vPerp = vSum / (float)vCount;

    // negate, as the sensor layout is inversed now (S1 is in the back)
    vPerp = -vPerp;

    return vPerp;
}

// ---- ISR handling ----------------------------------------------------

// Static trampolines simply forward to the instance method.
void IRSensors::isrSensor1Trampoline()
{
    if (_instance)
    {
        _instance->handleSensorTrigger(1, micros());
    }
}

void IRSensors::isrSensor2Trampoline()
{
    if (_instance)
    {
        _instance->handleSensorTrigger(2, micros());
    }
}

void IRSensors::isrSensor3Trampoline()
{
    if (_instance)
    {
        _instance->handleSensorTrigger(3, micros());
    }
}

// Called from ISRs whenever one sensor detects the line.
// sensorIndex: 1, 2 or 3 indicating which sensor fired.
// t_us: timestamp from micros() when the interrupt occurred.
void IRSensors::handleSensorTrigger(uint8_t sensorIndex, uint32_t t_us)
{
    // If this is the first trigger in a crossing, start a new measurement window.
    if (!_t1_valid && !_t2_valid && !_t3_valid)
    {
        _crossingStart_us = t_us;
    }
    else
    {
        // If the time since the first trigger exceeds the timeout, reset and
        // start a new crossing window from this trigger (faulty previous event).
        if ((uint32_t)(t_us - _crossingStart_us) > CROSSING_TIMEOUT_US)
        {
            _t1_valid = _t2_valid = _t3_valid = false;
            _crossingStart_us = t_us;
        }
    }

    // Store timestamp for corresponding sensor.
    if (sensorIndex == 1)
    {
        _t1_us = t_us;
        _t1_valid = true;
    }
    else if (sensorIndex == 2)
    {
        _t2_us = t_us;
        _t2_valid = true;
    }
    else if (sensorIndex == 3)
    {
        _t3_us = t_us;
        _t3_valid = true;
    }

    // If all three sensors have triggered within the timeout, we can
    // compute dt12_x, dt13_x, dt23_x and then angle + velocity.
    if (_t1_valid && _t2_valid && _t3_valid)
    {
        // Compute raw time differences in seconds.
        float t1 = (float)_t1_us * 1e-6f;
        float t2 = (float)_t2_us * 1e-6f;
        float t3 = (float)_t3_us * 1e-6f;

        // For the normalized dt*_x values we need a scaling factor.
        // Here we simply use a symmetric normalization based on the
        // maximum expected time difference (in seconds). You should
        // set this to a reasonable value for your hovercraft speed.
        const float dtMax = 0.010f; // example: 10 ms max difference

        float dt12_x = clamp1((t2 - t1) / dtMax);
        float dt13_x = clamp1((t3 - t1) / dtMax);
        float dt23_x = clamp1((t3 - t2) / dtMax);

        // Compute angle alpha from normalized dt-values.
        float alphaDeg = computeAlphaFromDt(dt12_x, dt13_x, dt23_x);

        // Compute perpendicular velocity using raw time differences.
        float dt12_raw = t2 - t1;
        float dt13_raw = t3 - t1;
        float dt23_raw = t3 - t2;

        float vPerp = computeVelocityPerpToLine(alphaDeg,
                                                _a,
                                                dt12_raw,
                                                dt13_raw,
                                                dt23_raw);

        _lastAlphaDeg = alphaDeg;
        _lastVelPerp = vPerp;
        _hasNewMeasurement = true;

        // Reset for next crossing event.
        _t1_valid = _t2_valid = _t3_valid = false;
    }
}