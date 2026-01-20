#include "hovercraft_variables.h"
// In here, all hovercraft variables/settings are defined that can be used to change the main parameters of
// the hovercraft without searching through all the code files

// *************************************************
// Define all IO-Pins (actual definitions)
// *************************************************
const int LED_PIN = LED_BUILTIN;

// Motor IO/DSHOT Pins
const gpio_num_t global_PIN_MOTOR_FL = GPIO_NUM_7;
const gpio_num_t global_PIN_MOTOR_FR = GPIO_NUM_43;
const gpio_num_t global_PIN_MOTOR_BL = GPIO_NUM_44;
const gpio_num_t global_PIN_MOTOR_BR = GPIO_NUM_8;

// Battery Monitoring Pins
const gpio_num_t global_PIN_BATTERY_VOLTAGE_MONITOR = GPIO_NUM_9;
const gpio_num_t global_PIN_BATTERY_CURRENT_MONITOR = GPIO_NUM_4;

// IR-Sensor Pins
const gpio_num_t global_PIN_IR_SENSOR_FL = GPIO_NUM_3; // Front-Left sensor
const gpio_num_t global_PIN_IR_SENSOR_FR = GPIO_NUM_2; // Front-Right sensor
const gpio_num_t global_PIN_IR_SENSOR_BM = GPIO_NUM_1; // Back sensor

// I2C Pins (GPIO5 = SDA, GPIO6 = SCL)
const gpio_num_t global_PIN_I2C_SDA = GPIO_NUM_5;
const gpio_num_t global_PIN_I2C_SCL = GPIO_NUM_6;

// *************************************************
// Battery related limits and variables(actual definitions)
// *************************************************
// Voltage divider (resistors) ratio: V_battery = V_pin * ratio
const float global_BatteryVoltage_VoltageDividerRatio = 0.3037f; // voltage divider ratio, depends on the resistors used for voltage divider, here 100kOhm/(100kOhm+220kOhm) = 0.3125 -> ajusted through measurement
const float global_BatteryCurrent_VoltageDividerRatio = 1.0f;    // current sensor voltage divider ratio, if any

// Betaflight-style current scale in units of [0.1 mV] / Amp.
// Example: scale=130 => 13 mV/A => Current(A) = Sensor_mV / 13
const float global_BatteryCurrent_SensorScaler_AmpsPerVolt = 130.0f; // from the ESC datasheet

const float global_BatteryVoltageLow_WarningLow = 7.2f;          // low battery indicator threshold, for 2s LiPo = 3.50V per cell
const float global_BatteryVoltageLow_MotorCutoffLow = 6.4f;      // motor cutoff threshold, for 2s LiPo = 3.10V per cell
const uint16_t global_BatteryVoltageLow_MotorCutoffSamples = 20; // motors are disabled if below cutoff for more than this many samples

// *************************************************
// Motor control variables (actual definitions)
// *************************************************
const float global_NegativeRpmScaleFactor = 2.0f;                                // reverse-direction compensation (applied to negative motor commands)
const float global_WebLiftPresetPercent_Array[] = {45.0f, 60.0f, 75.0f, 100.0f}; // lift presets for web UI selection
const size_t global_WebLiftPresetPercent_Array_len = sizeof(global_WebLiftPresetPercent_Array) / sizeof(global_WebLiftPresetPercent_Array[0]);
const int global_WebLiftPresetPercent_Array_startIndex = 1; // e.g. 1 -> first preset is 60% after startup (startup is always 0)
const float global_WebThrustPresetPercent = 60.0f;          // percent thrust, when in web UI "full forward" mode is 100%, this is still scaled with global_AllMotorsScalePercent!!

bool global_MotorsReversedFL = true;  // Flag: if true, all Front Left motor commands are reversed
bool global_MotorsReversedFR = false; // Flag: if true, all Front Right motor commands are reversed
bool global_MotorsReversedBL = true;  // Flag: if true, all Back Left motor commands are reversed
bool global_MotorsReversedBR = false; // Flag: if true, all Back Right motor commands are reversed

// overall motor power scaler to avoid overloading/burning motors, adjust depending on kV
const float global_AllMotorsScalePercent = 60.0f; // 40.0f for CraftLG (18 000kV), 60.0f for CraftENSMM (14 800kV)

// *************************************************
// Wifi SSID, PW, IP addresses
// *************************************************
// const char global_WifiApSsid[] = "µ-verCraft-II AP-LG";       // Craft LG
const char global_WifiApSsid[] = "µ-verCraft-II AP"; // Craft ENSMM
const char global_WifiApPassword[] = "Supmicrotech"; // minimum 8 chars for WPA2
// Note: WebUi/Website: 192.168.4.1
const uint16_t global_WebServerPort = 80;

// *************************************************
// Magnetometer calibration (hard-iron offsets)
// *************************************************
// Replace these values with the output of MagCalibrator tool found in 3_Software of this git repo
// Craft LG
// const int16_t global_MagOffsetX = -602; // Avg of -602, -613, -590
// const int16_t global_MagOffsetY = 399;  // Avg of 389, 373, 434
// const int16_t global_MagOffsetZ = -438; // Avg of -510, -202, -602

// Craft ENSMM
const int16_t global_MagOffsetX = -619;
const int16_t global_MagOffsetY = 989;
const int16_t global_MagOffsetZ = 44;

// *************************************************
// Gyro/IMU/Complementary filter settings
// *************************************************
const float global_ComplementaryFilter_yawAlpha = 0.9f; // Complementary filter alpha, high = trust gyro more, old 0.9f

// *************************************************
// Control loop constants (example, if you enable them)
// *************************************************
// Main control loop rate for controllers (Hz)
const float global_ControlLoopRate_Hz = 400.0f;

// *************************************************
// Rates = responsiveness of the sliders
// *************************************************
// Betaflight-style max yaw rate at full input deflection
// look up online "betaflight actual rates" for more information
// Also, there is a file under 3_Software/rates.html that allows to visualize the response curve
const float global_MaxYawRateSetpoint_dps = 2.0f * 360.0f; // 2 full rotations per second = 720 deg/s
const float global_YawRateExpo = 0.40f;                    // expo for yaw rate response curve
const float global_YawCenterSensitivity = 450.0f;          // sensitivity around center for small deflections

/// *************************************************
// PID controller constants
// *************************************************
// Yaw RATE PID (deg/s).
// Start conservative; tune on hardware.
// LuisCraft:
// const float global_YawRatePid_Kp = 2.0f;        // LuisCraft: 2.0f
// const float global_YawRatePid_Ki = 4.0f;        // LuisCraft: 4.0f
// const float global_YawRatePid_Kd = 0.04f;       // LuisCraft: 0.04f

// ENSMM Craft:
const float global_YawRatePid_Kp = 0.8f;  // ENSMM Craft: 1.0f
const float global_YawRatePid_Ki = 2.0f;  // ENSMM Craft: 2.0f
const float global_YawRatePid_Kd = 0.01f; // ENSMM Craft: 0.02f

// PID output is fed into MotorMixer::setDiffThrust() which expects [-100..100].
const float global_YawRatePid_OutputLimit = 100.0f;

// Integral clamp in same units as output.
const float global_YawRatePid_IntegratorLimit = 50.0f;

// *************************************************
// Heading (absolute yaw) controller constants
// *************************************************
// Outer loop: heading error (deg) -> yaw-rate setpoint (deg/s).
// Start with conservative gains; tune on hardware.
// Ziegler nicosl: Ku = 50, Pu = 1,5s => Kp 0,6*Ku=30, Ki=1.2*Ku/Pu=40, Kd=0.075*Ku*Pu=5
const float global_HeadingPid_Kp = 25.0f; // 6
const float global_HeadingPid_Ki = 1.1f;  // 0
const float global_HeadingPid_Kd = 0.3f;  // 0.3

// Limit yaw-rate request from heading-hold to keep it smooth.
const float global_HeadingPid_OutputLimit_dps = 100.0f;
const float global_HeadingPid_IntegratorLimit_dps = 45.0f;

// *************************************************
// Dimensions
// *************************************************
// IR Sensor distances in Isosceles triangle
const float global_IRSensorDistance_a_meters = 0.07f; // triangle has equal sides of length a
const float global_IRSensorDistance_b_meters = 0.06f; // triangle has base of length b

// sensor positions
// motor positions

// *************************************************
// Thresholds, limits, etc.
// *************************************************
// IR-line sensor thresholds
const float global_IRSensor_Threshold = 150.0f;     // black line over 150
const float global_IRSensor_Hysteresis = 5.0f;      // hysteresis for line detection
const float global_IRSensor_Timeout_us = 500000.0f; // timeout in microseconds for incomplete crossings (0.3 seconds)
