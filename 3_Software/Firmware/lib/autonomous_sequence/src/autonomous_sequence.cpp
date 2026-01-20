#include <autonomous_sequence.h>

#include <math.h>

void AutonomousSequence::requestStart()
{
    startRequested_ = true;
}

void AutonomousSequence::requestStop()
{
    stopRequested_ = true;
}

bool AutonomousSequence::isActive() const
{
    return state_ != State::Idle;
}

AutonomousSequence::State AutonomousSequence::state() const
{
    return state_;
}

bool AutonomousSequence::overrideThrust() const
{
    return overrideThrust_;
}

float AutonomousSequence::thrustOverride_percent() const
{
    return thrustOverridePercent_;
}

bool AutonomousSequence::wantsHeadingHold() const
{
    return wantsHeadingHold_;
}

float AutonomousSequence::headingTarget_deg() const
{
    return headingTargetDeg_;
}

bool AutonomousSequence::consumeExitRequest()
{
    if (!exitRequestLatched_)
    {
        return false;
    }
    exitRequestLatched_ = false;
    return true;
}

float AutonomousSequence::startHeading_deg() const
{
    return startHeadingDeg_;
}

float AutonomousSequence::wrap360(float deg)
{
    if (!isfinite(deg))
        return 0.0f;
    while (deg >= 360.0f)
        deg -= 360.0f;
    while (deg < 0.0f)
        deg += 360.0f;
    return deg;
}

float AutonomousSequence::deg2rad(float deg)
{
    return deg * (float)(M_PI / 180.0);
}

void AutonomousSequence::enterState(State s, uint32_t nowMs)
{
    state_ = s;
    stateStartMs_ = nowMs;
}

void AutonomousSequence::resetCalibration()
{
    sumSin_ = 0.0f;
    sumCos_ = 0.0f;
    sampleCount_ = 0;
}

void AutonomousSequence::accumulateHeading(float heading_deg)
{
    const float h = wrap360(heading_deg);
    const float r = deg2rad(h);
    sumSin_ += sinf(r);
    sumCos_ += cosf(r);
    sampleCount_++;
}

float AutonomousSequence::computeAverageHeading() const
{
    if (sampleCount_ == 0)
    {
        return startHeadingDeg_;
    }
    const float ang = atan2f(sumSin_, sumCos_) * (float)(180.0 / M_PI);
    return wrap360(ang);
}

void AutonomousSequence::update(uint32_t nowMs, float heading_deg, bool headingFresh, bool motorsEnabled)
{
    // Default outputs each tick
    overrideThrust_ = false;
    thrustOverridePercent_ = 0.0f;
    wantsHeadingHold_ = false;
    headingTargetDeg_ = 0.0f;

    // Apply stop request at any time.
    if (stopRequested_)
    {
        stopRequested_ = false;
        if (state_ != State::Idle)
        {
            enterState(State::ExitRequested, nowMs);
        }
    }

    // Start request handled only while idle.
    if (state_ == State::Idle)
    {
        if (startRequested_)
        {
            startRequested_ = false;
            resetCalibration();
            startHeadingDeg_ = wrap360(heading_deg);
            enterState(State::Calibrating, nowMs);
        }
        return;
    }

    // Safety: if motors are disarmed during thrust phases, stop and exit.
    if (!motorsEnabled)
    {
        if (state_ == State::HoldStart || state_ == State::TurnMinus90 || state_ == State::TurnMinus180)
        {
            enterState(State::ExitRequested, nowMs);
        }
    }

    const uint32_t elapsedMs = nowMs - stateStartMs_;

    // The real "SPS-like" programming for the competition sequence is defined here:
    // New sequence, it is no longer time based, this time it's event based:
    // Line-sequence is as follows: 0/blind/startLine -> 0 -> -90 -> -90 -> 0 -> 0 -> -90 -> -90 -> 0 -> 0 -> -90 -> -90 -> 0 -> 0 -> -90 -> -90 -> 0 -> 0 -> ... (continue loop)
    //[0°/blind] -> [0°/line detect] -> [-90°/line detect] -> [0°/line detect] -> [0°/line detect] -> [-90°/line detect] -> [-90°/line detect] -> [0°/line detect] -> exit
    /*
        New idea for real structure:
        - Calibrating: 2s standstill, average heading
        - WaitingForArm: wait until motorsEnabled==true

        - When "go":
        1.) [0°/blind]: hold direction "blind" for 0.5s with thrust 50% -> surpass the start line blindly
        2.) [0°/line detect]: hold direction "bilnd" until IR sensors detect line (thrust 45%), then
            "correction step": use the angle from the IR sensors, do math:
            newHeadingSetpoint = currentHeading - alpha - 0deg (currentAngle from Complementary filter, alpha from IR sensors)
        3.) [-90°/line detect]: continue as before, until line detected, then do correction step with -90deg, maths:
            newHeadingSetpoint = currentHeading - alpha - (-90deg)
        4.) [0°/line detect]: continue as before, until line detected, then do correction step with 0deg, maths:
            newHeadingSetpoint = currentHeading - alpha - 0deg
        ... repeat steps 3 and 4 until stop condition ...
    */

    // right here is the old time-based sequence, needs to be replaced with the new event-based sequence
    // time based was: calibrate, go straight 1.6s, turn -90 0.75s, turn -180 5s, exit
    switch (state_)
    {
    case State::Calibrating:
        // Stand still for 2s and average heading.
        overrideThrust_ = true;
        thrustOverridePercent_ = 0.0f;
        wantsHeadingHold_ = false;

        if (headingFresh)
        {
            accumulateHeading(heading_deg);
        }

        if (elapsedMs >= 2000)
        {
            startHeadingDeg_ = computeAverageHeading();
            if (motorsEnabled)
            {
                enterState(State::HoldStart, nowMs);
            }
            else
            {
                enterState(State::WaitingForArm, nowMs);
            }
        }
        break;

    case State::WaitingForArm:
        // Motors are expected to remain stopped here (caller keeps outputs at 0).
        overrideThrust_ = true;
        thrustOverridePercent_ = 0.0f;
        wantsHeadingHold_ = false;

        if (motorsEnabled)
        {
            enterState(State::HoldStart, nowMs);
        }
        break;

    case State::HoldStart:
        // Hold heading for 2s with thrust override.
        overrideThrust_ = true;
        thrustOverridePercent_ = 20.0f;
        wantsHeadingHold_ = true;
        headingTargetDeg_ = wrap360(startHeadingDeg_);

        if (elapsedMs >= 1600)
        {
            enterState(State::TurnMinus90, nowMs);
        }
        break;

    case State::TurnMinus90:
        overrideThrust_ = true;
        thrustOverridePercent_ = 20.0f;
        wantsHeadingHold_ = true;
        headingTargetDeg_ = wrap360(startHeadingDeg_ - 90.0f);

        if (elapsedMs >= 750)
        {
            enterState(State::TurnMinus180, nowMs);
        }
        break;

    case State::TurnMinus180:
        overrideThrust_ = true;
        thrustOverridePercent_ = 20.0f;
        wantsHeadingHold_ = true;
        headingTargetDeg_ = wrap360(startHeadingDeg_ - 180.0f);

        if (elapsedMs >= 5000)
        {
            enterState(State::ExitRequested, nowMs);
        }
        break;

    case State::ExitRequested:
        // One-shot exit request.
        overrideThrust_ = true;
        thrustOverridePercent_ = 0.0f;
        wantsHeadingHold_ = false;
        exitRequestLatched_ = true;
        enterState(State::Idle, nowMs);
        break;

    case State::Idle:
    default:
        break;
    }
}
