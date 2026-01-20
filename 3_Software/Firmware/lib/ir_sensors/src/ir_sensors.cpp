#include "ir_sensors.h"
#include <math.h>

// Provided by project globals (definition in hovercraft_variables.cpp).
extern const float global_IRSensor_Timeout_us;

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
        _crossingStart_us(0),
        _sensorWriteIdx(0),
        _sensorCount(0),
        _sampleQueue(nullptr)
{
        _armed[0] = _armed[1] = _armed[2] = true;
        _prevInit[0] = _prevInit[1] = _prevInit[2] = false;
        _prevValue[0] = _prevValue[1] = _prevValue[2] = 0;
}

// Initialize pins and queue.
void IRSensors::begin()
{
    if (_sampleQueue == nullptr)
    {
        _sampleQueue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(Sample));
    }
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

bool IRSensors::enqueueSample(const uint8_t values[3], const uint32_t t_us[3])
{
    if (_sampleQueue == nullptr)
    {
        return false;
    }

    Sample s;
    for (int i = 0; i < 3; ++i)
    {
        s.v[i] = values[i];
        s.t[i] = t_us[i];
    }
    //Serial.println("Enqueued sample");
    return xQueueSend(_sampleQueue, &s, 0) == pdPASS;
}

bool IRSensors::dequeueSample(uint8_t values[3], uint32_t t_us[3], TickType_t waitTicks)
{
    if (_sampleQueue == nullptr)
    {
        return false;
    }

    Sample s;
    if (xQueueReceive(_sampleQueue, &s, waitTicks) != pdPASS)
    {
        return false;
    }

    for (int i = 0; i < 3; ++i)
    {
        values[i] = s.v[i];
        t_us[i] = s.t[i];
    }
    return true;
}

void IRSensors::processQueue(float threshold, float hysteresis)
{
    uint8_t values[3];
    uint32_t times[3];
    // uint32_t processedCount = 0;

    // Keep consuming until queue is empty to avoid backlog when producer runs faster.
    while (dequeueSample(values, times, 0))
    {
        detectLine(values, times, threshold, hysteresis);
        // processedCount++;
        // Serial.println("Dequeued sample: " + String(processedCount));
    }
}

void IRSensors::detectLine(const uint8_t values[3], const uint32_t t_us[3], float threshold, float hysteresis)
{
    (void)hysteresis;

    // Rising-edge detection per sensor using previous sample + simple re-arm.
    for (int s = 0; s < 3; ++s)
    {
        uint8_t v = values[s];

        if (!_prevInit[s])
        {
            _prevValue[s] = v;
            _prevInit[s] = true;
            // Set armed if we start below the threshold.
            _armed[s] = (v < threshold);
            continue;
        }

        // Re-arm when below threshold.
        if (v < threshold)
        {
            _armed[s] = true;
        }

        bool rising = _armed[s] && (_prevValue[s] < threshold) && (v >= threshold);
        if (rising)
        {
            uint32_t ts = t_us[s];
            _armed[s] = false; // lock out until re-armed

            if (s == 0)
            {
                _t1_us = ts;
                _t1_valid = true;
            }
            else if (s == 1)
            {
                _t2_us = ts;
                _t2_valid = true;
            }
            else if (s == 2)
            {
                _t3_us = ts;
                _t3_valid = true;
            }

            // Debug: log rising edge detection for this sensor.
            Serial.printf("[IRSensors] rise s=%d v=%u ts=%lu\n", s, (unsigned)v, (unsigned long)ts);
        }

        _prevValue[s] = v;
    }

    // Once all three have toggled, compute alpha and speed using isosceles geometry:
    // positions: S1=BM at (0,-h), S2=FL at (-b/2,0), S3=FR at (b/2,0), base toward front.
    // h = sqrt(a^2 - (b/2)^2)
    if (_t1_valid && _t2_valid && _t3_valid)
    {
        const float a = _a;
        const float b = _b;
        const float halfBase = b * 0.5f;
        if (a > halfBase)
        {
            const float h = sqrtf(a * a - halfBase * halfBase);

            // Raw time differences in seconds
            float dt12 = ((float)_t2_us - (float)_t1_us) * 1e-6f; // FL - BM
            float dt13 = ((float)_t3_us - (float)_t1_us) * 1e-6f; // FR - BM
            float dt23 = ((float)_t3_us - (float)_t2_us) * 1e-6f; // FR - FL

            // Heading angle (alpha) from tan(alpha) = (2 h dt23) / (b (dt12 + dt13))
            float denom = b * (dt12 + dt13);
            if (fabsf(denom) > 1e-9f)
            {
                float alphaRad = atanf((2.0f * h * dt23) / denom);
                float alphaDeg = RAD2DEG(alphaRad);
                // Wrap to (-180, 180]
                if (alphaDeg > 180.0f)
                {
                    alphaDeg -= 360.0f;
                }
                else if (alphaDeg <= -180.0f)
                {
                    alphaDeg += 360.0f;
                }

                // Speed perpendicular to the line: v = (b * sin(alpha)) / dt23
                float vPerp = NAN;
                if (fabsf(dt23) > 1e-9f)
                {
                    float vMag = (b * sinf(alphaRad)) / dt23; // magnitude from geometry

                    // Determine sign from who crosses first: front avg vs back
                    // lead < 0 -> front (FL/FR) earlier -> forward -> positive
                    float t1 = (float)_t1_us * 1e-6f; // BM
                    float t2 = (float)_t2_us * 1e-6f; // FL
                    float t3 = (float)_t3_us * 1e-6f; // FR
                    float frontMeanMinusBack = 0.5f * (t2 + t3) - t1;
                    float sign = (frontMeanMinusBack < 0.0f) ? 1.0f : -1.0f;
                    vPerp = sign * fabsf(vMag);
                }

                _lastAlphaDeg = alphaDeg;
                _lastVelPerp = vPerp;
                _hasNewMeasurement = true;
            }
        }

        // Reset for next crossing event.
        _t1_valid = _t2_valid = _t3_valid = false;
    }
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

    return vPerp;
}