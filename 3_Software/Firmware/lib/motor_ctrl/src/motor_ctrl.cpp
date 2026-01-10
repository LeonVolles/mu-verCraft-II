#include "motor_ctrl.h"
#include "DShotESC.h"

// Create DShotESC instances for each motor
static DShotESC escFL;
static DShotESC escFR;
static DShotESC escBL;
static DShotESC escBR;

MotorCtrl::MotorCtrl(float generalMotorPowerScalerPercent)
    : pGeneralMotorPowerScalerPercent(generalMotorPowerScalerPercent)
{
    // Initialize motor speed percentages to 0
    pPercentFL = 0.0;
    pPercentFR = 0.0;
    pPercentBL = 0.0;
    pPercentBR = 0.0;
}

void MotorCtrl::init(gpio_num_t fl_pin, gpio_num_t fr_pin, gpio_num_t bl_pin, gpio_num_t br_pin, bool pSetReversedFL, bool pSetReversedFR, bool pSetReversedBL, bool pSetReversedBR)
{
    // Initialize ESC for Front Left motor using provided IO pin and fixed RMT channel 0
    escFL.install(fl_pin, RMT_CHANNEL_0);
    escFL.init();
    escFL.setReversed(pSetReversedFL);
    escFL.set3DMode(true);
    for (int i = 0; i < 5; i++)
    {
        escFL.beep(i);
    }

    // Initialize ESC for Front Right motor using provided IO pin and fixed RMT channel 1
    escFR.install(fr_pin, RMT_CHANNEL_1);
    escFR.init();
    escFR.setReversed(pSetReversedFR);
    escFR.set3DMode(true);
    for (int i = 0; i < 5; i++)
    {
        escFR.beep(i);
    }

    // Initialize ESC for Back Left motor using provided IO pin and fixed RMT channel 2
    escBL.install(bl_pin, RMT_CHANNEL_2);
    escBL.init();
    escBL.setReversed(pSetReversedBL);
    escBL.set3DMode(true);
    for (int i = 0; i < 5; i++)
    {
        escBL.beep(i);
    }

    // Initialize ESC for Back Right motor using provided IO pin and fixed RMT channel 3
    escBR.install(br_pin, RMT_CHANNEL_3);
    escBR.init();
    escBR.setReversed(pSetReversedBR);
    escBR.set3DMode(true);
    for (int i = 0; i < 5; i++)
    {
        escBR.beep(i);
    }

    // Ensure Motors start at 0RPM
    pPercentFL = 0.0;
    pPercentFR = 0.0;
    pPercentBL = 0.0;
    pPercentBR = 0.0;
}

void MotorCtrl::setFrontLeftPercent(float pct)
{
    pPercentFL = pct;
    escFL.sendThrottle3D((int16_t)(pPercentFL * pGeneralMotorPowerScalerPercent)); // Send current throttle values to each ESC. Throttle is provided as integer.
}

void MotorCtrl::setFrontRightPercent(float pct)
{
    pPercentFR = pct;
    escFR.sendThrottle3D((int16_t)(pPercentFR * pGeneralMotorPowerScalerPercent));
}

void MotorCtrl::setBackLeftPercent(float pct)
{
    pPercentBL = pct;
    escBL.sendThrottle3D((int16_t)(pPercentBL * pGeneralMotorPowerScalerPercent));
}

void MotorCtrl::setBackRightPercent(float pct)
{
    pPercentBR = pct;
    escBR.sendThrottle3D((int16_t)(pPercentBR * pGeneralMotorPowerScalerPercent));
}

void MotorCtrl::setAllPercent(float fl, float fr, float bl, float br)
{
    pPercentFL = fl;
    pPercentFR = fr;
    pPercentBL = bl;
    pPercentBR = br;

    escFL.sendThrottle3D((int16_t)(pPercentFL * pGeneralMotorPowerScalerPercent));
    escFR.sendThrottle3D((int16_t)(pPercentFR * pGeneralMotorPowerScalerPercent));
    escBL.sendThrottle3D((int16_t)(pPercentBL * pGeneralMotorPowerScalerPercent));
    escBR.sendThrottle3D((int16_t)(pPercentBR * pGeneralMotorPowerScalerPercent));
}

float MotorCtrl::getFrontLeftPercent()
{
    return pPercentFL;
}

float MotorCtrl::getFrontRightPercent()
{
    return pPercentFR;
}

float MotorCtrl::getBackLeftPercent()
{
    return pPercentBL;
}

float MotorCtrl::getBackRightPercent()
{
    return pPercentBR;
}