#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <Arduino.h>
#include "driver/gpio.h" // for GPIO_NUM_X when building with Arduino-ESP32

class DShotESC;

class MotorCtrl
{
private:
    struct KickState
    {
        int16_t lastSentThrottle = 0; // last throttle value sent to ESC (-999..999)
        bool active = false;          // currently in kick phase
        int16_t pendingDesired = 0;   // latest desired throttle while kick is active
        int16_t kickThrottle = 0;     // throttle being used during kick phase
        uint32_t kickStartMs = 0;     // millis() when kick started
    };

    float pPercentFL;
    float pPercentFR;
    float pPercentBL;
    float pPercentBR;

    bool pSetReversedFL;
    bool pSetReversedFR;
    bool pSetReversedBL;
    bool pSetReversedBR;

    float pGeneralMotorPowerScalerPercent; // Overall power scaler (0 to 100%), this is used to prevent smoking the motors.

    // Global safety gate. If false, ALL motors are forced to 0 throttle.
    // This is intended as an emergency override that must win over any mixer/setpoint logic.
    bool pMotorsEnabled;

    KickState pKickFL;
    KickState pKickFR;
    KickState pKickBL;
    KickState pKickBR;

    void sendThrottle3D_WithKick(DShotESC &esc, KickState &state, int16_t desiredThrottle);
    void applyEmergencyOff();

public:
    MotorCtrl(float generalMotorPowerScalerPercent);

    // Master enable (emergency stop): when disabled, all motors are immediately commanded to 0.
    void setMotorsEnabled(bool enabled);
    bool motorsEnabled() const;
    void EmergencyOff();

    // Immediately stop the lift motors (front left/right) without affecting rear thrust motors.
    // Note: The global emergency gate (setMotorsEnabled(false)) still wins over everything.
    void applyLiftOff();

    // Initialization with 4 IO pins
    void init(gpio_num_t fl_pin, gpio_num_t fr_pin, gpio_num_t bl_pin, gpio_num_t br_pin, bool setReversedFL = false, bool pSetReversedFR = false, bool pSetReversedBL = false, bool pSetReversedBR = false);

    void tempForDebug_SetFL_directly(int16_t throttle);  // TEMP for debug, set FL motor directly with throttle value between -999 and 999
    void tempForDebug_SetAll_directly(int16_t throttle); // TEMP for debug, set all motors directly with same throttle value between -999 and 999

    // Individual motor control (-100% to +100%)
    void setFrontLeftPercent(float pct);
    void setFrontRightPercent(float pct);
    void setBackLeftPercent(float pct);
    void setBackRightPercent(float pct);

    // Set all motors at once
    void setAllPercent(float fl, float fr, float bl, float br);

    // Get current motor percentages
    float getFrontLeftPercent();
    float getFrontRightPercent();
    float getBackLeftPercent();
    float getBackRightPercent();
};

#endif // MOTOR_CTRL_H