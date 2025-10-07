#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <Arduino.h>

class MotorCtrl {
private:
    float frontLeftPercent;
    float frontRightPercent;
    float backLeftPercent;
    float backRightPercent;
    
public:
    MotorCtrl();
    
    // Initialization
    void init();
    
    // Update function (called in main loop)
    void update();
    
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