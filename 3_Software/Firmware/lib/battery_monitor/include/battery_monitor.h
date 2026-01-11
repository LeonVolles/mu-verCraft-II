#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

class BatteryMonitor
{
private:
    int _voltageAdcPin;
    int _currentAdcPin;

    float _voltageDividerRatio; // V_battery = V_pin * ratio
    float _currentDividerRatio; // V_sensor = V_pin * ratio

    // Betaflight-style scale in units of [0.1 mV] / Amp.
    // Example: 130 => 13mV/A => Current(A) = Sensor_mV / 13
    float _betaflightScale_0p1mVPerAmp;
    float _currentOffset_mV;

    float _voltage_V;
    float _current_A;
    float _used_mAh;

    uint32_t _lastUpdateMs;

    float readPinMilliVolts(int pin) const;

public:
    BatteryMonitor();

    // Configure voltage/current ADC pins and calibration.
    void init(int voltageAdcPin,
              int currentAdcPin,
              float voltageDividerRatio,
              float currentDividerRatio,
              float betaflightScale_0p1mVPerAmp);

    // Optional: if your current sensor isn't 0mV at 0A, you can remove an offset.
    void setCurrentOffset_mV(float offset_mV);

    // Update voltage/current and integrate used mAh (uses millis() delta_t).
    void update();

    // Battery measurements
    float getVoltage();
    float getCurrent();
    float getMAH();
    void setMAH(float mAh);
};

#endif // BATTERY_MONITOR_H