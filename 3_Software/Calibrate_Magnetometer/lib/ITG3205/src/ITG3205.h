/*
 * ITG3205 minimal driver (local project file)
 *
 * Created by: Goebes, Luis
 * Project: SimpleMPUTesting
 * Note: This libary was self written
 *
 * Main functionality:
 * - Probes the I2C bus for an ITG-3200/ITG3205-compatible gyro at 0x68/0x69 (ACK-based), with optional I2C speed fallback.
 * - Initializes the gyro (reset, clock source, sample rate, DLPF/full-scale config, interrupts off).
 * - Calibrates stationary bias by averaging N samples (keep board still during calibration).
 * - Reads raw gyro + temperature and converts to deg/s plus an approximate temperature in °C.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

class ITG3205
{
public:
    static constexpr uint8_t DefaultAddr0 = 0x68;
    static constexpr uint8_t DefaultAddr1 = 0x69;

    ITG3205() = default;

    // Step 1: probe device on the I2C bus.
    // Returns true if a device ACKs at addr0 or addr1. Stores chosen address.
    bool probe(TwoWire &wire, uint8_t addr0 = DefaultAddr0, uint8_t addr1 = DefaultAddr1, uint32_t busHz = 400000,
               uint32_t fallbackHz = 100000);

    // Step 2: configure registers (reset, clock, sample rate, DLPF+FS).
    bool initialize();

    // Step 3: calibrate bias (keep board still).
    bool calibrate(uint16_t samples, Stream *log = &Serial);

    // Read values in deg/s, with stored bias removed. Also returns approximate temp.
    bool read(float &gxDps, float &gyDps, float &gzDps, float &tempC);

    // Optional raw read.
    bool readRaw(int16_t &gx, int16_t &gy, int16_t &gz, int16_t &tempRaw);

    bool isReady() const { return _ready; }
    uint8_t address() const { return _addr; }
    uint8_t whoAmI() const { return _whoAmI; }

private:
    static constexpr uint8_t REG_WHO_AM_I = 0x00;
    static constexpr uint8_t REG_SMPLRT_DIV = 0x15;
    static constexpr uint8_t REG_DLPF_FS = 0x16;
    static constexpr uint8_t REG_INT_CFG = 0x17;
    static constexpr uint8_t REG_TEMP_OUT_H = 0x1B;
    static constexpr uint8_t REG_PWR_MGM = 0x3E;

    static constexpr float SensLsbPerDps = 14.375f; // 2000 dps full-scale

    bool ping(uint8_t addr);
    bool write8(uint8_t reg, uint8_t value);
    bool readN(uint8_t reg, uint8_t *buf, size_t len);

    TwoWire *_wire = nullptr;
    uint8_t _addr = 0;
    uint8_t _whoAmI = 0xFF;
    bool _ready = false;

    float _biasX = 0.0f;
    float _biasY = 0.0f;
    float _biasZ = 0.0f;
};
