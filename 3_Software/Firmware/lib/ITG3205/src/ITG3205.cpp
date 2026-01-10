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

#include "ITG3205.h"

bool ITG3205::ping(uint8_t addr)
{
    if (_wire == nullptr)
        return false;
    _wire->beginTransmission(addr);
    return _wire->endTransmission() == 0;
}

bool ITG3205::write8(uint8_t reg, uint8_t value)
{
    if (_wire == nullptr || _addr == 0)
        return false;
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(value);
    return _wire->endTransmission() == 0;
}

bool ITG3205::readN(uint8_t reg, uint8_t *buf, size_t len)
{
    if (_wire == nullptr || _addr == 0)
        return false;
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0)
        return false;
    size_t got = _wire->requestFrom((int)_addr, (int)len);
    if (got != len)
        return false;
    for (size_t i = 0; i < len; i++)
        buf[i] = _wire->read();
    return true;
}

bool ITG3205::probe(TwoWire &wire, uint8_t addr0, uint8_t addr1, uint32_t busHz, uint32_t fallbackHz)
{
    _wire = &wire;
    _ready = false;
    _addr = 0;
    _whoAmI = 0xFF;

    // Try at requested bus speed.
    _wire->setClock(busHz);
    if (ping(addr0))
    {
        _addr = addr0;
    }
    else if (ping(addr1))
    {
        _addr = addr1;
    }
    else
    {
        // Try a slower speed.
        _wire->setClock(fallbackHz);
        delay(10);
        if (ping(addr0))
            _addr = addr0;
        else if (ping(addr1))
            _addr = addr1;
        // Restore.
        _wire->setClock(busHz);
    }

    if (_addr == 0)
        return false;

    uint8_t who = 0xFF;
    if (readN(REG_WHO_AM_I, &who, 1))
        _whoAmI = who;

    return true;
}

bool ITG3205::initialize()
{
    if (_wire == nullptr || _addr == 0)
        return false;

    // Reset device
    if (!write8(REG_PWR_MGM, 0x80))
        return false;
    delay(10);

    // Use PLL with X gyro as clock (more stable than internal)
    if (!write8(REG_PWR_MGM, 0x01))
        return false;

    // Sample rate divider: internal 1kHz / (div+1)
    // With DLPF on: internal is 1kHz. div=9 -> 100Hz
    if (!write8(REG_SMPLRT_DIV, 9))
        return false;

    // DLPF + Full scale: FS_SEL=3 (2000dps), DLPF_CFG=3 (42Hz)
    if (!write8(REG_DLPF_FS, 0x1B))
        return false;

    // Interrupt config: disable interrupts
    if (!write8(REG_INT_CFG, 0x00))
        return false;

    _ready = true;
    return true;
}

bool ITG3205::readRaw(int16_t &gx, int16_t &gy, int16_t &gz, int16_t &tempRaw)
{
    if (!_ready)
        return false;
    uint8_t buf[8] = {0};
    if (!readN(REG_TEMP_OUT_H, buf, sizeof(buf)))
        return false;

    tempRaw = (int16_t)((buf[0] << 8) | buf[1]);
    gx = (int16_t)((buf[2] << 8) | buf[3]);
    gy = (int16_t)((buf[4] << 8) | buf[5]);
    gz = (int16_t)((buf[6] << 8) | buf[7]);
    return true;
}

bool ITG3205::calibrate(uint16_t samples, Stream *log)
{
    if (!_ready)
        return false;

    if (log != nullptr)
        log->printf("GYRO: calibrating (%u samples). Keep board still...\n", samples);

    long sumX = 0, sumY = 0, sumZ = 0;
    uint16_t got = 0;
    int16_t gx, gy, gz, t;

    for (uint16_t i = 0; i < samples; i++)
    {
        if (readRaw(gx, gy, gz, t))
        {
            sumX += gx;
            sumY += gy;
            sumZ += gz;
            got++;
        }
        delay(5);
    }

    if (got == 0)
    {
        if (log != nullptr)
            log->println("GYRO: calibration failed (no samples)");
        return false;
    }

    _biasX = (float)sumX / (float)got;
    _biasY = (float)sumY / (float)got;
    _biasZ = (float)sumZ / (float)got;

    if (log != nullptr)
        log->printf("GYRO: bias raw: X=%.1f Y=%.1f Z=%.1f\n", _biasX, _biasY, _biasZ);

    return true;
}

bool ITG3205::read(float &gxDps, float &gyDps, float &gzDps, float &tempC)
{
    gxDps = NAN;
    gyDps = NAN;
    gzDps = NAN;
    tempC = NAN;

    int16_t rgx, rgy, rgz, rtemp;
    if (!readRaw(rgx, rgy, rgz, rtemp))
        return false;

    gxDps = ((float)rgx - _biasX) / SensLsbPerDps;
    gyDps = ((float)rgy - _biasY) / SensLsbPerDps;
    gzDps = ((float)rgz - _biasZ) / SensLsbPerDps;

    // Common ITG-3200 family approximation:
    // Temp(C) = 35 + (raw + 13200) / 280
    tempC = 35.0f + (((float)rtemp + 13200.0f) / 280.0f);

    return true;
}
