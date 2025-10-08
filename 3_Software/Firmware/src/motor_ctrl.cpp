#include "motor_ctrl.h"
#include "DShotESC.h"

// Create DShotESC instances for each motor
static DShotESC escFL;
static DShotESC escFR;
static DShotESC escBL;
static DShotESC escBR;

MotorCtrl::MotorCtrl()
{
    frontLeftPercent = 0.0;
    frontRightPercent = 0.0;
    backLeftPercent = 0.0;
    backRightPercent = 0.0;
}

void MotorCtrl::init(gpio_num_t fl_pin, gpio_num_t fr_pin, gpio_num_t bl_pin, gpio_num_t br_pin)
{
    // Initialize ESC for Front Left motor using provided IO pin and fixed RMT channel 0
    escFL.install(fl_pin, RMT_CHANNEL_0);
    escFL.init();
    escFL.setReversed(false);
    escFL.set3DMode(true);
    for (int i = 0; i < 5; i++)
    {
        escFL.beep(i);
    }

    // Initialize ESC for Front Right motor using provided IO pin and fixed RMT channel 1
    escFR.install(fr_pin, RMT_CHANNEL_1);
    escFR.init();
    escFR.setReversed(false);
    escFR.set3DMode(true);
    for (int i = 0; i < 5; i++)
    {
        escFR.beep(i);
    }

    // Initialize ESC for Back Left motor using provided IO pin and fixed RMT channel 2
    escBL.install(bl_pin, RMT_CHANNEL_2);
    escBL.init();
    escBL.setReversed(false);
    escBL.set3DMode(true);
    for (int i = 0; i < 5; i++)
    {
        escBL.beep(i);
    }

    // Initialize ESC for Back Right motor using provided IO pin and fixed RMT channel 3
    escBR.install(br_pin, RMT_CHANNEL_3);
    escBR.init();
    escBR.setReversed(false);
    escBR.set3DMode(true);
    for (int i = 0; i < 5; i++)
    {
        escBR.beep(i);
    }

    // Ensure Motors start at 0RPM
    frontLeftPercent = 0.0;
    frontRightPercent = 0.0;
    backLeftPercent = 0.0;
    backRightPercent = 0.0;
}

void MotorCtrl::update()
{
    // Send current throttle values to each ESC. Throttle is provided as integer.
    escFL.sendThrottle3D((int16_t)frontLeftPercent);
    escFR.sendThrottle3D((int16_t)frontRightPercent);
    escBL.sendThrottle3D((int16_t)backLeftPercent);
    escBR.sendThrottle3D((int16_t)backRightPercent);
}

void MotorCtrl::setFrontLeftPercent(float pct)
{
    frontLeftPercent = pct;
}

void MotorCtrl::setFrontRightPercent(float pct)
{
    frontRightPercent = pct;
}

void MotorCtrl::setBackLeftPercent(float pct)
{
    backLeftPercent = pct;
}

void MotorCtrl::setBackRightPercent(float pct)
{
    backRightPercent = pct;
}

void MotorCtrl::setAllPercent(float fl, float fr, float bl, float br)
{
    frontLeftPercent = fl;
    frontRightPercent = fr;
    backLeftPercent = bl;
    backRightPercent = br;
}

float MotorCtrl::getFrontLeftPercent()
{
    return frontLeftPercent;
}

float MotorCtrl::getFrontRightPercent()
{
    return frontRightPercent;
}

float MotorCtrl::getBackLeftPercent()
{
    return backLeftPercent;
}

float MotorCtrl::getBackRightPercent()
{
    return backRightPercent;
}