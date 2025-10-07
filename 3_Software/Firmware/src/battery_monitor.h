#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

class BatteryMonitor {
private:
    int adcPin;
    float voltage;
    float current;
    float mAh;
    float vRef;      // Reference voltage for ADC
    float r1, r2;    // Voltage divider resistors
    
public:
    BatteryMonitor();
    
    // Initialization with ADC pin
    void init(int adcPin);
    
    // Update measurements (call in main loop)
    void update();
    
    // Battery measurements
    float getVoltage();
    float getCurrent();
    float getMAH();
    void setMAH(float mAh);
    
    // Calibration
    void setCalibration(float vRef, float r1, float r2);
    void getCalibration(float* vRef, float* r1, float* r2);
};

#endif // BATTERY_MONITOR_H