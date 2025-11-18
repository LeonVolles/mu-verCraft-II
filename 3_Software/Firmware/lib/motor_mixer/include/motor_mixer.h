#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <Arduino.h>
#include "motor_ctrl.h"

class MotorMixer
{
public:
    // Mixer operates on an existing MotorCtrl instance
    MotorMixer(MotorCtrl &motorCtrl);

    void init();

    // Input ranges:
    // lift:        0   to 100
    // thrust:     -100 to 100
    // diffThrust: -100 to 100 (0 means equal power for both rear motors)
    void setLift(float lift);
    void setThrust(float thrust);
    void setDiffThrust(float diffThrust);

    float getLift() const;
    float getThrust() const;
    float getDiffThrust() const;

private:
    MotorCtrl &m_motorCtrl;

    float m_lift;
    float m_thrust;
    float m_diffThrustBalance;

    // Internal function that calculates and applies motor outputs
    void updateOutputs();
};

#endif // MOTOR_MIXER_H
