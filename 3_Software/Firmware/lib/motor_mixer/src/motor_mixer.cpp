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

    // Rear motors get thrust plus differential thrust
    float thrust = constrain(m_thrust, -100.0f, 100.0f);
    float diff = constrain(m_diffThrustBalance, -100.0f, 100.0f);

    // Absolute value defines total rear power, sign defines direction
    float base = fabs(thrust);                         // 0..100
    float direction = (thrust >= 0.0f) ? 1.0f : -1.0f; // forward or reverse

    // Differential thrust is applied proportionally
    // diff = 0   -> both rear motors get the same power
    // diff = 100 -> left motor full power, right motor zero
    // diff = -100 -> right motor full power, left motor zero
    float d = diff / 100.0f;

    float backLeft = base * (1.0f + d);
    float backRight = base * (1.0f - d);

    // Clamp to valid range 0..100 before applying direction
    backLeft = constrain(backLeft, 0.0f, 100.0f);
    backRight = constrain(backRight, 0.0f, 100.0f);

    // Apply common direction to both rear motors
    backLeft *= direction;
    backRight *= direction;

    // Send to MotorCtrl
    m_motorCtrl.setAllPercent(frontLeft, frontRight, backLeft, backRight);
}
