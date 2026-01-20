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

float AutonomousSequence::lastLineAlpha_deg() const
{
    return lastLineAlphaDeg_;
}

float AutonomousSequence::lastLineVelocityPerp_mps() const
{
    return lastLineVelPerp_mps_;
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

void AutonomousSequence::update(uint32_t nowMs,
                                float heading_deg,
                                bool headingFresh,
                                bool motorsEnabled,
                                bool lineEventFresh,
                                float lineAlpha_deg,
                                float lineVPerp_mps,
                                float headingAtLine_deg)
{
    // Default outputs each tick
    overrideThrust_ = false;
    thrustOverridePercent_ = 0.0f;
    wantsHeadingHold_ = false;
    // NOTE: Do NOT reset headingTargetDeg_ here.
    // It is the persistent heading-hold setpoint that must survive across ticks while we are
    // waiting for the next line event.

    // Latch the latest line telemetry for possible future strategies/debug.
    // lineEventFresh is expected to be a one-shot (edge) signal: true only on the tick where
    // the IR module reports a NEW line crossing event.
    if (lineEventFresh)
    {
        lastLineAlphaDeg_ = lineAlpha_deg;
        lastLineVelPerp_mps_ = lineVPerp_mps;
    }

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

    // Safety: if motors are disarmed while the sequence is actively commanding motion, stop and exit.
    if (!motorsEnabled)
    {
        if (state_ == State::StartBlind ||
            state_ == State::DriveStraight_FirstSector ||
            state_ == State::DriveCurveMinus90_FirstSector ||
            state_ == State::DriveCurveMinus90_SecondSector ||
            state_ == State::DriveStraight_SecondSector)
        {
            enterState(State::ExitRequested, nowMs);
        }
    }

    const uint32_t elapsedMs = nowMs - stateStartMs_;

    /*
       The goal is:
        // The real "SPS-like" programming for the competition sequence is defined here:
        // Line-sequence is like the following sectors: 0/blind/jumpStartLine -> -90 -> -90 -> 0 -> 0 -> -90 -> -90 -> 0 -> 0 -> -90 -> -90 -> 0 -> 0 -> -90 -> -90 -> 0 -> 0 -> ... (continue loop)
        //[blind/hold] -> [waitForLine/-90°] -> [waitForLine/-90°] -> [waitForLine/0°] -> [waitForLine/0°] -> [waitForLine/-90°] -> [waitForLine/-90°] -> [waitForLine/0°] -> -> [waitForLine/0°] -> …
        Wee keep:
        - Calibrating: 2s standstill, average heading
        - WaitingForArm: wait until motorsEnabled==true

        - When "go":
        1.) [waitForLine/0°/blind]: hold direction "blind" for a short period with a little higher thrust than later -> surpass the start line blindly

        2.) DriveCurveMinus90_FirstSector: wait until IR sensors detect line, now we will start the first -90° turn. We do this by the "correction step": use the angle from the IR sensors, do math:
            newHeadingSetpoint = currentHeading - alpha - 90deg (currentAngle from Complementary filter, alpha from IR sensors)

        3.) DriveCurveMinus90_SecondSector: wait until IR sensors detect line , now we will start the second -90° turn, again with the correction step with -90deg, maths:
            newHeadingSetpoint = currentHeading - alpha -90deg

        4.) DriveStraight_FirstSector: wait until IR sensors detect line, this times we want to try to drive as perpendicular to the line as possible. For this we use the formula:
            newHeadingSetpoint = currentHeading - alpha + 0deg

        5.) DriveStraight_SecondSector: wait until IR sensors detect line, this times we want to try to drive as perpendicular to the line as possible. For this we use the formula:
            newHeadingSetpoint = currentHeading - alpha + 0deg

        6.) loop back to DriveCurveMinus90_FirstSector: wait until IR sensors detect line, now we will start the first -90° turn. We do this by the "correction step": use the angle from the IR sensors, do math:
            newHeadingSetpoint = currentHeading - alpha - 90deg (currentAngle from Complementary filter, alpha from IR sensors)

        ... repeat until motors are turned of, or M-button is deactivated ...
    */

    // This is to compensate a slight right drift of the hovercraft, to cancel out sideways drift from before
    // This is applied to all "curves"
    float driftAngleDeg_firstSector = 57.0f; // this value neeeds adjustment depending on speed, but this is for later
    float driftAngleDeg_secondSector = 7.5f; // this is smaller, cause the creaft is slower

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
                headingTargetDeg_ = wrap360(startHeadingDeg_);
                enterState(State::StartBlind, nowMs);
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
            headingTargetDeg_ = wrap360(startHeadingDeg_);
            enterState(State::StartBlind, nowMs);
        }
        break;

    case State::StartBlind:
        // 1) [0°/blind]: hold direction "blind" for 1.0s with thrust 30% to surpass the start line.
        Serial.println("1.) StartBlind, aveaged heading: " + String(startHeadingDeg_));
        overrideThrust_ = true;
        thrustOverridePercent_ = 30.0f;
        wantsHeadingHold_ = true;
        headingTargetDeg_ = wrap360(startHeadingDeg_);

        // Drive blind for 1.0s, then switch to DriveStraight_SecondSector to wait for the first line event that starts the first -90° curve.
        if (elapsedMs >= 1000)
        {
            // Because we did not arrive at the line triggering the first curve, we give to DriveStraight_SecondSector so we continue straight until the first line event that starts the first -90° curve.
            Serial.println("   Transition From blind drive to wainting for first line event: DriveStraight_SecondSector");
            enterState(State::DriveStraight_SecondSector, nowMs);
        }
        break;

    case State::DriveStraight_FirstSector:
        // We continue to drive straight (0°) first sector until a new line event.
        overrideThrust_ = true;
        thrustOverridePercent_ = 10.0f;
        wantsHeadingHold_ = true;
        headingTargetDeg_ = wrap360(headingTargetDeg_);

        // When arriving at a line, we set the setpoint to "driving straight 0° second sector" and give to DriveStraight_SecondSector
        if (lineEventFresh)
        {
            // Line event -> end of this straight sector.
            // Recompute heading target for the next straight sector (0°).
            const float alpha = lineAlpha_deg;
            const float headingAtLine = headingAtLine_deg;

            // newHeadingSetpoint = currentHeading - alpha + 0deg
            headingTargetDeg_ = wrap360(headingAtLine - alpha);
            Serial.println("   Line detected! Next: NewH = headingAtLine - alpha (0deg)" +
                           String(headingTargetDeg_) + " = " + String(headingAtLine) + " - " + String(alpha));
            Serial.println("   Transition to DriveStraight_SecondSector");
            enterState(State::DriveStraight_SecondSector, nowMs);
        }
        break;

    case State::DriveStraight_SecondSector:
        // We continue to drive straight (0°) second sector until a new line event.
        overrideThrust_ = true;
        thrustOverridePercent_ = 10.0f;
        wantsHeadingHold_ = true;
        headingTargetDeg_ = wrap360(headingTargetDeg_);

        // When arriving at a line, we set the setpoint to "turning -90°" and give to DriveCurveMinus90_FirstSector
        if (lineEventFresh)
        {
            const float alpha = lineAlpha_deg;
            const float headingAtLine = headingAtLine_deg;
            // Line event -> end of straight segment, next segment is the first -90° sector.

            // newHeadingSetpoint = currentHeading - alpha - 90deg - abs(driftAngleDeg_firstSector)
            headingTargetDeg_ = wrap360(headingAtLine - alpha - 90.0f - abs(driftAngleDeg_firstSector));
            Serial.println("   Line detected! Next: NewH = headingAtLine - alpha - 90deg - abs(driftAngleDeg_firstSector)" +
                           String(headingTargetDeg_) + " = " + String(headingAtLine) + " - " + String(alpha) + " - 90 - " + String(abs(driftAngleDeg_firstSector)));
            Serial.println("   Transition to DriveCurveMinus90_FirstSector");
            enterState(State::DriveCurveMinus90_FirstSector, nowMs);
        }
        break;

    case State::DriveCurveMinus90_FirstSector:
        // Continue to drive curve -90° first sector until a new line event.
        overrideThrust_ = true;
        thrustOverridePercent_ = 10.0f;
        wantsHeadingHold_ = true;
        headingTargetDeg_ = wrap360(headingTargetDeg_);

        // When arriving at a line, we set the setpoint to "turning -90° second sector" and give to DriveCurveMinus90_SecondSector
        if (lineEventFresh)
        {
            // Line event -> end of first -90° curve sector.
            const float alpha = lineAlpha_deg;
            const float headingAtLine = headingAtLine_deg;

            // newHeadingSetpoint = currentHeading - alpha - 90deg - abs(driftAngleDeg_secondSector)
            headingTargetDeg_ = wrap360(headingAtLine - alpha - 90.0f - abs(driftAngleDeg_secondSector));
            Serial.println("   Line detected! Next: NewH = headingAtLine - alpha - 90deg - abs(driftAngleDeg_secondSector)" +
                           String(headingTargetDeg_) + " = " + String(headingAtLine) + " - " + String(alpha) + " - 90 - " + String(abs(driftAngleDeg_secondSector)));
            Serial.println("   Transition to DriveCurveMinus90_SecondSector");
            enterState(State::DriveCurveMinus90_SecondSector, nowMs);
        }
        break;

    case State::DriveCurveMinus90_SecondSector:
        // We continue to drive the second curve -90° second sector until a new line event.
        overrideThrust_ = true;
        thrustOverridePercent_ = 10.0f;
        wantsHeadingHold_ = true;
        headingTargetDeg_ = wrap360(headingTargetDeg_);

        // When arriving at a line, we set the setpoint to "driving straight 0° first sector" and give to DriveStraight_FirstSector
        if (lineEventFresh)
        {
            // Line event -> end of -90° segment, next segment returns to straight (0°).
            const float alpha = lineAlpha_deg;
            const float headingAtLine = headingAtLine_deg;

            // newHeadingSetpoint = currentHeading - alpha + 0deg
            headingTargetDeg_ = wrap360(headingAtLine - alpha);
            Serial.println("   Line detected! Next: NewH = headingAtLine - alpha (0deg)" +
                           String(headingTargetDeg_) + " = " + String(headingAtLine) + " - " + String(alpha));
            Serial.println("   Transition to DriveStraight_FirstSector");
            enterState(State::DriveStraight_FirstSector, nowMs);
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
