#include "battery_monitor.h"

#if defined(ARDUINO_ARCH_ESP32)
// ADC attenuation/resolution helpers live in Arduino-ESP32 core.
#include <esp32-hal-adc.h>
#endif

static inline float clamp_minf(float v, float minv)
{
    return (v < minv) ? minv : v;
}

BatteryMonitor::BatteryMonitor()
    : _voltageAdcPin(-1),
      _currentAdcPin(-1),
      _voltageDividerRatio(1.0f),
      _currentDividerRatio(1.0f),
      _betaflightScale_0p1mVPerAmp(0.0f),
      _currentOffset_mV(0.0f),
      _voltage_V(0.0f),
      _current_A(0.0f),
      _used_mAh(0.0f),
      _lastUpdateMs(0)
{
}

float BatteryMonitor::readPinMilliVolts(int pin) const
{
    if (pin < 0)
    {
        return 0.0f;
    }

#if defined(ARDUINO_ARCH_ESP32)
    // Arduino-ESP32 provides calibrated mV readings.
    return (float)analogReadMilliVolts(pin);
#else
    // Generic Arduino fallback (assumes 10-bit ADC, 3.3V reference).
    const int raw = analogRead(pin);
    return (float)raw * (3300.0f / 1023.0f);
#endif
}

void BatteryMonitor::init(int voltageAdcPin,
                          int currentAdcPin,
                          float voltageDividerRatio,
                          float currentDividerRatio,
                          float betaflightScale_0p1mVPerAmp)
{
    _voltageAdcPin = voltageAdcPin;
    _currentAdcPin = currentAdcPin;
    _voltageDividerRatio = (voltageDividerRatio > 0.0f) ? voltageDividerRatio : 1.0f;
    _currentDividerRatio = (currentDividerRatio > 0.0f) ? currentDividerRatio : 1.0f;
    _betaflightScale_0p1mVPerAmp = betaflightScale_0p1mVPerAmp;

#if defined(ARDUINO_ARCH_ESP32)
    // Reasonable defaults for ESP32-S3: 12-bit resolution and wide input range.
    analogReadResolution(12);
    if (_voltageAdcPin >= 0)
    {
        analogSetPinAttenuation(_voltageAdcPin, ADC_11db);
    }
    if (_currentAdcPin >= 0)
    {
        analogSetPinAttenuation(_currentAdcPin, ADC_11db);
    }
#endif

    _lastUpdateMs = millis();
}

void BatteryMonitor::setCurrentOffset_mV(float offset_mV)
{
    _currentOffset_mV = offset_mV;
}

void BatteryMonitor::update()
{
    const uint32_t nowMs = millis();
    uint32_t dtMs = 0;
    if (_lastUpdateMs != 0)
    {
        dtMs = nowMs - _lastUpdateMs;
    }
    _lastUpdateMs = nowMs;

    const float vPin_mV = readPinMilliVolts(_voltageAdcPin);
    const float iPin_mV = readPinMilliVolts(_currentAdcPin);

    const float vBatt_mV = vPin_mV / _voltageDividerRatio;
    _voltage_V = vBatt_mV / 1000.0f;

    // Convert Betaflight-style scale to mV/A.
    // scale is in [0.1mV]/A => mV/A = scale * 0.1
    const float mvPerAmp = _betaflightScale_0p1mVPerAmp * 0.1f;
    const float sensor_mV = iPin_mV * _currentDividerRatio;
    const float sensorAdj_mV = clamp_minf(sensor_mV - _currentOffset_mV, 0.0f);

    if (mvPerAmp > 0.0f)
    {
        _current_A = sensorAdj_mV / mvPerAmp;
    }
    else
    {
        _current_A = 0.0f;
    }

    // Integrate used capacity (mAh). Works even for larger update intervals.
    const float dt_s = (float)dtMs / 1000.0f;
    if (dt_s > 0.0f)
    {
        _used_mAh += (_current_A * dt_s / 3600.0f) * 1000.0f;
    }
}

float BatteryMonitor::getVoltage()
{
    return _voltage_V;
}

float BatteryMonitor::getCurrent()
{
    return _current_A;
}

float BatteryMonitor::getMAH()
{
    return _used_mAh;
}

void BatteryMonitor::setMAH(float mAh)
{
    _used_mAh = mAh;
}