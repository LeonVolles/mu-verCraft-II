#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <Arduino.h>
#include "driver/gpio.h" // for GPIO_NUM_X when building with Arduino-ESP32

class MotorCtrl
{
private:
    float pPercentFL;
    float pPercentFR;
    float pPercentBL;
    float pPercentBR;

    bool pSetReversedFL;
    bool pSetReversedFR;
    bool pSetReversedBL;
    bool pSetReversedBR;

    float pGeneralMotorPowerScalerPercent; // Overall power scaler (0 to 100%), this is used to prevent smoking the motors.

public:
    MotorCtrl(float generalMotorPowerScalerPercent);

    // Initialization with 4 IO pins
    void init(gpio_num_t fl_pin, gpio_num_t fr_pin, gpio_num_t bl_pin, gpio_num_t br_pin, bool setReversedFL = false, bool pSetReversedFR = false, bool pSetReversedBL = false, bool pSetReversedBR = false);

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