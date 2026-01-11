#include "motor_ctrl.h"
#include "DShotESC.h"

// Create DShotESC instances for each motor
DShotESC escFL;
DShotESC escFR;
DShotESC escBL;
DShotESC escBR;

static constexpr int16_t kDShot3D_MaxThrottle = 999;
static constexpr int16_t kDShot3D_KickAbsThrottle = 50; // ~5% of 999
static constexpr uint32_t kDShot3D_KickDurationMs = 20; // per user observation

static inline int16_t clampThrottle(int32_t throttle)
{
    if (throttle > kDShot3D_MaxThrottle)
        return kDShot3D_MaxThrottle;
    if (throttle < -kDShot3D_MaxThrottle)
        return -kDShot3D_MaxThrottle;
    return (int16_t)throttle;
}

static inline int16_t abs16(int16_t v)
{
    return (v < 0) ? (int16_t)-v : v;
}

void MotorCtrl::sendThrottle3D_WithKick(DShotESC &esc, KickState &state, int16_t desiredThrottle)
{
    desiredThrottle = clampThrottle(desiredThrottle);

    // If desired is 0: always send 0 and reset kick state.
    if (desiredThrottle == 0)
    {
        esc.sendThrottle3D(0);
        state.lastSentThrottle = 0;
        state.active = false;
        state.pendingDesired = 0;
        state.kickThrottle = 0;
        state.kickStartMs = 0;
        return;
    }

    const int16_t kickThrottle = (desiredThrottle > 0) ? kDShot3D_KickAbsThrottle : (int16_t)-kDShot3D_KickAbsThrottle;

    // Start a kick phase if we were at standstill and the new command is "large".
    if (!state.active && state.lastSentThrottle == 0 && abs16(desiredThrottle) > kDShot3D_KickAbsThrottle)
    {
        state.active = true;
        state.pendingDesired = desiredThrottle;
        state.kickThrottle = kickThrottle;
        state.kickStartMs = millis();

        esc.sendThrottle3D(state.kickThrottle);
        state.lastSentThrottle = state.kickThrottle;
        return;
    }

    // If kick phase is active, keep sending kick for ~20ms, then switch to desired.
    if (state.active)
    {
        state.pendingDesired = desiredThrottle; // always keep latest command

        const uint32_t elapsed = millis() - state.kickStartMs;
        if (elapsed >= kDShot3D_KickDurationMs)
        {
            esc.sendThrottle3D(state.pendingDesired);
            state.lastSentThrottle = state.pendingDesired;
            state.active = false;
            return;
        }

        esc.sendThrottle3D(state.kickThrottle);
        state.lastSentThrottle = state.kickThrottle;
        return;
    }

    // Normal case: just send desired.
    esc.sendThrottle3D(desiredThrottle);
    state.lastSentThrottle = desiredThrottle;
}

void MotorCtrl::applyEmergencyOff()
{
    // Force all motors to 0 throttle immediately (independent of any mixer/setpoint logic).
    sendThrottle3D_WithKick(escFL, pKickFL, 0);
    sendThrottle3D_WithKick(escFR, pKickFR, 0);
    sendThrottle3D_WithKick(escBL, pKickBL, 0);
    sendThrottle3D_WithKick(escBR, pKickBR, 0);

    // Keep internal state consistent with outputs.
    pPercentFL = 0.0f;
    pPercentFR = 0.0f;
    pPercentBL = 0.0f;
    pPercentBR = 0.0f;

    // Reset kick state so a later re-enable starts cleanly.
    pKickFL = KickState{};
    pKickFR = KickState{};
    pKickBL = KickState{};
    pKickBR = KickState{};
}

MotorCtrl::MotorCtrl(float generalMotorPowerScalerPercent)
    : pGeneralMotorPowerScalerPercent(generalMotorPowerScalerPercent)
{
    // Initialize motor speed percentages to 0
    pPercentFL = 0.0;
    pPercentFR = 0.0;
    pPercentBL = 0.0;
    pPercentBR = 0.0;

    pSetReversedFL = false;
    pSetReversedFR = false;
    pSetReversedBL = false;
    pSetReversedBR = false;

    // Safety: default to disabled until the control logic explicitly arms.
    pMotorsEnabled = false;
}

void MotorCtrl::setMotorsEnabled(bool enabled)
{
    pMotorsEnabled = enabled;
    if (!pMotorsEnabled)
    {
        applyEmergencyOff();
    }
}

bool MotorCtrl::motorsEnabled() const
{
    return pMotorsEnabled;
}

void MotorCtrl::EmergencyOff()
{
    setMotorsEnabled(false);
}

void MotorCtrl::applyLiftOff()
{
    // Stop lift motors immediately (front left/right).
    // We intentionally do not touch rear motors here.
    pPercentFL = 0.0f;
    pPercentFR = 0.0f;

    sendThrottle3D_WithKick(escFL, pKickFL, 0);
    sendThrottle3D_WithKick(escFR, pKickFR, 0);
}

void MotorCtrl::init(gpio_num_t fl_pin, gpio_num_t fr_pin, gpio_num_t bl_pin, gpio_num_t br_pin, bool pSetReversedFL, bool pSetReversedFR, bool pSetReversedBL, bool pSetReversedBR)
{
    // Store reversal flags
    this->pSetReversedFL = pSetReversedFL;
    this->pSetReversedFR = pSetReversedFR;
    this->pSetReversedBL = pSetReversedBL;
    this->pSetReversedBR = pSetReversedBR;

    // Initialize ESC for Front Left motor using provided IO pin and fixed RMT channel 0
    escFL.install(fl_pin, RMT_CHANNEL_0);
    escFL.init();
    escFL.setReversed(false); // this does not seem to work, so we handle reversal in MotorCtrl manually
    escFL.set3DMode(true);

    // Initialize ESC for Front Right motor using provided IO pin and fixed RMT channel 1
    escFR.install(fr_pin, RMT_CHANNEL_1);
    escFR.init();
    escFR.setReversed(false); // this does not seem to work, so we handle reversal in MotorCtrl manually
    escFR.set3DMode(true);

    // Initialize ESC for Back Left motor using provided IO pin and fixed RMT channel 2
    escBL.install(bl_pin, RMT_CHANNEL_2);
    escBL.init();
    escBL.setReversed(false); // this does not seem to work, so we handle reversal in MotorCtrl manually
    escBL.set3DMode(true);

    // Initialize ESC for Back Right motor using provided IO pin and fixed RMT channel 3
    escBR.install(br_pin, RMT_CHANNEL_3);
    escBR.init();
    escBR.setReversed(false); // this does not seem to work, so we handle reversal in MotorCtrl manually
    escBR.set3DMode(true);

    // Many ESCs require a brief period of *zero throttle* after init (and/or after beeps)
    // before they accept non-zero commands (arming behavior). The standalone test sketch
    // does this implicitly because it starts by sending motorSpeed=0.
    // 500 loops are really needed, do not reduce!!!
    delay(100);
    for (int i = 0; i < 500; i++)
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

    // Reset kick state
    pKickFL = KickState{};
    pKickFR = KickState{};
    pKickBL = KickState{};
    pKickBR = KickState{};

    // Ensure outputs are 0 and the safety gate starts in the "disabled" state.
    pMotorsEnabled = false;
    applyEmergencyOff();
}

void MotorCtrl::tempForDebug_SetFL_directly(int16_t throttle)
{
    if (!pMotorsEnabled)
    {
        applyEmergencyOff();
        return;
    }
    sendThrottle3D_WithKick(escFL, pKickFL, throttle);
}

void MotorCtrl::tempForDebug_SetAll_directly(int16_t throttle)
{
    if (!pMotorsEnabled)
    {
        applyEmergencyOff();
        return;
    }
    sendThrottle3D_WithKick(escFL, pKickFL, throttle);
    sendThrottle3D_WithKick(escFR, pKickFR, throttle);
    sendThrottle3D_WithKick(escBL, pKickBL, throttle);
    sendThrottle3D_WithKick(escBR, pKickBR, throttle);
}

// Pct is in percent (-100.0 to +100.0)
// Throttle value to the ESC is between -999 and 999
void MotorCtrl::setFrontLeftPercent(float pct)
{
    if (!pMotorsEnabled)
    {
        // Emergency override: always force 0 throttle.
        pPercentFL = 0.0f;
        sendThrottle3D_WithKick(escFL, pKickFL, 0);
        return;
    }
    pPercentFL = pct;

    // Rescale from -100..100% to -999..999 for DShot3D
    float throttle_on999scale = 999.0f / 100.0f * pPercentFL; // Convert percent to -999 to 999 scale

    // Apply general motor power scaler
    float throttle_afterGeneralMotorPowerScalerPercent = throttle_on999scale * pGeneralMotorPowerScalerPercent / 100.0f; // Apply general motor power scaler

    // If flag "inverted" is set for this motor, invert the throttle command
    if (pSetReversedFL)
    {
        throttle_afterGeneralMotorPowerScalerPercent = -throttle_afterGeneralMotorPowerScalerPercent;
    }

    sendThrottle3D_WithKick(escFL, pKickFL, (int16_t)throttle_afterGeneralMotorPowerScalerPercent);
}

void MotorCtrl::setFrontRightPercent(float pct)
{
    if (!pMotorsEnabled)
    {
        pPercentFR = 0.0f;
        sendThrottle3D_WithKick(escFR, pKickFR, 0);
        return;
    }
    pPercentFR = pct;

    // Rescale from -100..100% to -999..999 for DShot3D
    float throttle_on999scale = 999.0f / 100.0f * pPercentFR; // Convert percent to -999 to 999 scale

    // Apply general motor power scaler
    float throttle_afterGeneralMotorPowerScalerPercent = throttle_on999scale * pGeneralMotorPowerScalerPercent / 100.0f; // Apply general motor power scaler

    // If flag "inverted" is set for this motor, invert the throttle command
    if (pSetReversedFR)
    {
        throttle_afterGeneralMotorPowerScalerPercent = -throttle_afterGeneralMotorPowerScalerPercent;
    }

    sendThrottle3D_WithKick(escFR, pKickFR, (int16_t)throttle_afterGeneralMotorPowerScalerPercent);
}

void MotorCtrl::setBackLeftPercent(float pct)
{
    if (!pMotorsEnabled)
    {
        pPercentBL = 0.0f;
        sendThrottle3D_WithKick(escBL, pKickBL, 0);
        return;
    }
    pPercentBL = pct;

    // Rescale from -100..100% to -999..999 for DShot3D
    float throttle_on999scale = 999.0f / 100.0f * pPercentBL; // Convert percent to -999 to 999 scale

    // Apply general motor power scaler
    float throttle_afterGeneralMotorPowerScalerPercent = throttle_on999scale * pGeneralMotorPowerScalerPercent / 100.0f; // Apply general motor power scaler

    // If flag "inverted" is set for this motor, invert the throttle command
    if (pSetReversedBL)
    {
        throttle_afterGeneralMotorPowerScalerPercent = -throttle_afterGeneralMotorPowerScalerPercent;
    }

    sendThrottle3D_WithKick(escBL, pKickBL, (int16_t)throttle_afterGeneralMotorPowerScalerPercent);
}

void MotorCtrl::setBackRightPercent(float pct)
{
    if (!pMotorsEnabled)
    {
        pPercentBR = 0.0f;
        sendThrottle3D_WithKick(escBR, pKickBR, 0);
        return;
    }
    pPercentBR = pct;

    // Rescale from -100..100% to -999..999 for DShot3D
    float throttle_on999scale = 999.0f / 100.0f * pPercentBR; // Convert percent to -999 to 999 scale

    // Apply general motor power scaler
    float throttle_afterGeneralMotorPowerScalerPercent = throttle_on999scale * pGeneralMotorPowerScalerPercent / 100.0f; // Apply general motor power scaler

    // If flag "inverted" is set for this motor, invert the throttle command
    if (pSetReversedBR)
    {
        throttle_afterGeneralMotorPowerScalerPercent = -throttle_afterGeneralMotorPowerScalerPercent;
    }

    sendThrottle3D_WithKick(escBR, pKickBR, (int16_t)throttle_afterGeneralMotorPowerScalerPercent);
}

void MotorCtrl::setAllPercent(float fl, float fr, float bl, float br)
{
    if (!pMotorsEnabled)
    {
        applyEmergencyOff();
        return;
    }
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