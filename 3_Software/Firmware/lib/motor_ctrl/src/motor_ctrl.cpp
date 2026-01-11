#include "motor_ctrl.h"
#include "DShotESC.h"

// Create DShotESC instances for each motor
DShotESC escFL;
DShotESC escFR;
DShotESC escBL;
DShotESC escBR;

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

    // Initialize ESC for Front Right motor using provided IO pin and fixed RMT channel 1
    escFR.install(fr_pin, RMT_CHANNEL_1);
    escFR.init();
    escFR.setReversed(pSetReversedFR);
    escFR.set3DMode(true);

    // Initialize ESC for Back Left motor using provided IO pin and fixed RMT channel 2
    escBL.install(bl_pin, RMT_CHANNEL_2);
    escBL.init();
    escBL.setReversed(pSetReversedBL);
    escBL.set3DMode(true);

    // Initialize ESC for Back Right motor using provided IO pin and fixed RMT channel 3
    escBR.install(br_pin, RMT_CHANNEL_3);
    escBR.init();
    escBR.setReversed(pSetReversedBR);
    escBR.set3DMode(true);

    // Beep all ESCs to indicate successful initialization
    for (int i = 0; i < 5; i++)
    {
        escFL.beep(i);
        escFR.beep(i);
        escBL.beep(i);
        escBR.beep(i);
    }

    // Many ESCs require a brief period of *zero throttle* after init (and/or after beeps)
    // before they accept non-zero commands (arming behavior). The standalone test sketch
    // does this implicitly because it starts by sending motorSpeed=0.
    for (int i = 0; i < 50; i++)
    {
        escFL.sendThrottle3D(0);
        escFR.sendThrottle3D(0);
        escBL.sendThrottle3D(0);
        escBR.sendThrottle3D(0);
        delay(20);
    }

    // Ensure Motors start at 0RPM
    pPercentFL = 0.0;
    pPercentFR = 0.0;
    pPercentBL = 0.0;
    pPercentBR = 0.0;
}

void MotorCtrl::tempForDebug_SetFL_directly(int16_t throttle)
{
    throttle = (int16_t)constrain((int)throttle, -999, 999);
    escFL.sendThrottle3D(throttle);
}

void MotorCtrl::tempForDebug_SetAll_directly(int16_t throttle)
{
    throttle = (int16_t)constrain((int)throttle, -999, 999);
    escFL.sendThrottle3D(throttle);
    escFR.sendThrottle3D(throttle);
    escBL.sendThrottle3D(throttle);
    escBR.sendThrottle3D(throttle);
}

// Pct is in percent (-100.0 to +100.0)
// Throttle value to the ESC is between -999 and 999
void MotorCtrl::setFrontLeftPercent(float pct)
{
    pPercentFL = pct;

    // Rescale from -100..100% to -999..999 for DShot3D
    float throttle_on999scale = 999.0f / 100.0f * pPercentFL; // Convert percent to -999 to 999 scale

    // Apply general motor power scaler
    float throttle_afterGeneralMotorPowerScalerPercent = throttle_on999scale * pGeneralMotorPowerScalerPercent / 100.0f; // Apply general motor power scaler

    // Send to ESC
    escFL.sendThrottle3D((int16_t)(throttle_afterGeneralMotorPowerScalerPercent)); // Send current throttle values to each ESC. Throttle is provided as integer.
}

void MotorCtrl::setFrontRightPercent(float pct)
{
    pPercentFR = pct;

    // Rescale from -100..100% to -999..999 for DShot3D
    float throttle_on999scale = 999.0f / 100.0f * pPercentFR; // Convert percent to -999 to 999 scale

    // Apply general motor power scaler
    float throttle_afterGeneralMotorPowerScalerPercent = throttle_on999scale * pGeneralMotorPowerScalerPercent / 100.0f; // Apply general motor power scaler

    // Send to ESC
    escFR.sendThrottle3D((int16_t)(throttle_afterGeneralMotorPowerScalerPercent));
}

void MotorCtrl::setBackLeftPercent(float pct)
{
    pPercentBL = pct;

    // Rescale from -100..100% to -999..999 for DShot3D
    float throttle_on999scale = 999.0f / 100.0f * pPercentBL; // Convert percent to -999 to 999 scale
    // Apply general motor power scaler
    float throttle_afterGeneralMotorPowerScalerPercent = throttle_on999scale * pGeneralMotorPowerScalerPercent / 100.0f; // Apply general motor power scaler
    // Send to ESC
    escBL.sendThrottle3D((int16_t)(throttle_afterGeneralMotorPowerScalerPercent));
}

void MotorCtrl::setBackRightPercent(float pct)
{
    pPercentBR = pct;

    // Rescale from -100..100% to -999..999 for DShot3D
    float throttle_on999scale = 999.0f / 100.0f * pPercentBR; // Convert percent to -999 to 999 scale

    // Apply general motor power scaler
    float throttle_afterGeneralMotorPowerScalerPercent = throttle_on999scale * pGeneralMotorPowerScalerPercent / 100.0f; // Apply general motor power scaler

    Serial.println("BR Throttle: " + String(throttle_afterGeneralMotorPowerScalerPercent) + " Int16 value: " + String((int16_t)(throttle_afterGeneralMotorPowerScalerPercent)));
    // Send to ESC
    escBR.sendThrottle3D((int16_t)(throttle_afterGeneralMotorPowerScalerPercent));
}

void MotorCtrl::setAllPercent(float fl, float fr, float bl, float br)
{
    pPercentFL = fl;
    pPercentFR = fr;
    pPercentBL = bl;
    pPercentBR = br;

    setFrontLeftPercent(fl);
    setFrontRightPercent(fr);
    setBackLeftPercent(bl);
    setBackRightPercent(br);
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