#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// OVERVIEW:
// This class handles three infrared line sensors mounted around the hovercraft.
// It:
// 1) captures precise timestamps (micros) when each sensor crosses a line,
// 2) computes the time differences dt*_x needed for angle estimation,
// 3) estimates the angle between hovercraft and line (alpha),
// 4) estimates the hovercraft velocity component perpendicular to the line,
// 5) stores the last valid angle and velocity for later readout via getters.
//
// Assumptions:
// - Sensors form an equilateral triangle around the center with side length a.
// - Hovercraft does not yaw significantly during one line crossing.
// - All three sensors detect the same line within a short time window.

class IRSensors
{
public:
    // Constructor: you must pass sensor pins and sensor distance a,b (meters).
    IRSensors(int pin1, int pin2, int pin3, float sensorDistance_a, float sensorDistance_b);

    // Initialize pins and attach interrupts. Must be called in setup().
    void begin();

    // Returns last computed angle (degrees) between hovercraft heading
    // and the line. Returns NAN if no valid measurement has been computed.
    float getAlphaToLine() const;

    // Returns last computed velocity perpendicular to the line (m/s).
    // Returns NAN if no valid measurement has been computed.
    float getVelocityPerpToLine() const;

    // Returns true if a new complete measurement (angle + velocity)
    // has been computed since the last call to consumeNewMeasurement().
    bool hasNewMeasurement() const;

    // Call this after reading angle/velocity to clear the "new data" flag.
    void consumeNewMeasurement();

    // Process one set of samples (values and common timestamp per sensor) for
    // hysteresis crossings, angle, and speed estimation. No buffering.
    void detectLine(const uint8_t values[3], const uint32_t t_us[3], float threshold, float hysteresis);

    // Drain all queued samples and run detectLine on each until queue is empty.
    void processQueue(float threshold, float hysteresis);

    // Producer/consumer helpers for inter-task handoff of ADC samples.
    bool enqueueSample(const uint8_t values[3], const uint32_t t_us[3]);
    bool dequeueSample(uint8_t values[3], uint32_t t_us[3], TickType_t waitTicks);

private:
    // Pins for the three IR sensors.
    int _pin1;
    int _pin2;
    int _pin3;

    // Distance between neighboring sensors (meters) distance a: sides,b: base between sensors for an isosceles triangle
    float _a;
    float _b;

    // Last computed angle (degrees) and perpendicular velocity (m/s).
    float _lastAlphaDeg;
    float _lastVelPerp;
    bool _hasNewMeasurement;

    // Raw timestamps (micros) when each sensor detected the line.
    volatile uint32_t _t1_us;
    volatile uint32_t _t2_us;
    volatile uint32_t _t3_us;

    // Flags to indicate which sensors have already been triggered
    // for the current crossing.
    volatile bool _t1_valid;
    volatile bool _t2_valid;
    volatile bool _t3_valid;

    // Time window (microseconds) within which all three sensors must
    // fire to be considered a valid single crossing event.
    static constexpr uint32_t CROSSING_TIMEOUT_US = 5000; // ~5 ms

    // Timestamp of the first trigger in the current crossing.
    volatile uint32_t _crossingStart_us;

    // Buffers for external sensor values and times.
    static constexpr uint16_t SENSOR_BUFFER_SIZE = 1000;
    volatile float _sensorValues[3][SENSOR_BUFFER_SIZE];
    volatile uint32_t _sensorTimes[3][SENSOR_BUFFER_SIZE];
    volatile uint16_t _sensorWriteIdx;
    volatile uint16_t _sensorCount;
    bool _sensorAbove[3];

    // Sample queue for task-safe producer/consumer between fast ADC poller and slower processor.
    struct Sample
    {
        float v[3];
        uint32_t t[3];
    };
    static constexpr uint16_t SAMPLE_QUEUE_LEN = 1000;
    QueueHandle_t _sampleQueue;

    // Internal helpers
    static float clamp1(float v);
    static float normalizeDeg(float a);
    static float computeAlphaFromDt(float dt12_x, float dt13_x, float dt23_x);
    static float computeVelocityPerpToLine(float alphaDeg,
                                           float a,
                                           float dt12,
                                           float dt13,
                                           float dt23);

    // Called from ISRs when a sensor sees a line.
    void handleSensorTrigger(uint8_t sensorIndex, uint32_t t_us);

    // Static ISR trampolines required by attachInterrupt().
    static void isrSensor1Trampoline();
    static void isrSensor2Trampoline();
    static void isrSensor3Trampoline();

    // Pointer to the single global instance used by ISRs.
    static IRSensors *_instance;
};

#endif // IR_SENSORS_H