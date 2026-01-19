#include "motor_mixer.h"

MotorMixer::MotorMixer(MotorCtrl &motorCtrl)
    : m_motorCtrl(motorCtrl),
      m_lift(0.0f),
      m_thrust(0.0f),
      m_diffThrustBalance(0.0f)
{
}

void MotorMixer::init()
{
    // Set all motors to zero initially
    m_lift = 0.0f;
    m_thrust = 0.0f;
    m_diffThrustBalance = 0.0f;
    updateOutputs();
}

void MotorMixer::setLift(float lift)
{
    // Ensure 0 to 100
    m_lift = constrain(lift, 0.0f, 100.0f);
    updateOutputs();
}

void MotorMixer::setThrust(float thrust)
{
    // Ensure -100 to 100
    m_thrust = constrain(thrust, -100.0f, 100.0f);
    updateOutputs();
}

void MotorMixer::setDiffThrust(float diffThrust)
{
    // Ensure -100 to 100
    m_diffThrustBalance = constrain(diffThrust, -100.0f, 100.0f);
    updateOutputs();
}

void MotorMixer::setLiftThrustDiff(float lift, float thrust, float diffThrust)
{
    // Set all three at once
    m_lift = constrain(lift, 0.0f, 100.0f);
    m_thrust = constrain(thrust, -100.0f, 100.0f);
    m_diffThrustBalance = constrain(diffThrust, -100.0f, 100.0f);
    updateOutputs();
}

float MotorMixer::getLift() const
{
    return m_lift;
}

float MotorMixer::getThrust() const
{
    return m_thrust;
}

float MotorMixer::getDiffThrust() const
{
    return m_diffThrustBalance;
}

void MotorMixer::updateOutputs()
{
    // Front motors: only lift, both equal
    float lift = constrain(m_lift, 0.0f, 100.0f);
    float frontLeft = lift;
    float frontRight = lift;

    // Rear motors: thrust plus ABSOLUTE differential
    // - Keeps the *average* rear output equal to `thrust`.
    // - Allows turning on the spot when `thrust == 0` by driving one motor forward and the other reverse.
    // - Each motor output is allowed in the full range -100..+100.
    float thrust = constrain(m_thrust, -100.0f, 100.0f);
    float diff = constrain(m_diffThrustBalance, -100.0f, 100.0f);

    // Absolute differential authority (in percent). Tune as needed.
    constexpr float diffAbsMaxPercent = 30.0f;
    float delta = (diff / 100.0f) * diffAbsMaxPercent;

    // diff = 100  -> left = thrust + 30, right = thrust - 30
    // diff = -100 -> left = thrust - 30, right = thrust + 30
    float backLeft = thrust + delta;
    float backRight = thrust - delta;

    backLeft = constrain(backLeft, -100.0f, 100.0f);
    backRight = constrain(backRight, -100.0f, 100.0f);

    // Rear-motor deadband: avoid jitter around 0, but keep it small enough
    // so heading-hold can still apply gentle counter-torque near the setpoint.
    constexpr float rearDeadbandPercent = 2.0f;
    if (fabs(backLeft) < rearDeadbandPercent)
    {
        backLeft = 0.0f;
    }
    if (fabs(backRight) < rearDeadbandPercent)
    {
        backRight = 0.0f;
    }
    // Send to MotorCtrl
    m_motorCtrl.setAllPercent(frontLeft, frontRight, backLeft, backRight);
}
