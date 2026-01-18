#ifndef HOVERCRAFT_VARIABLES_H
#define HOVERCRAFT_VARIABLES_H

// In here, all hovercraft variables/settings are defined that can be used to change the main parameters of
// the hovercraft without searching through all the code files

#include <Arduino.h>     // for LED_BUILTIN
#include "driver/gpio.h" // for gpio_num_t

// *************************************************
// Define all IO-Pins
// *************************************************
// LED Pin
// Declaration, the actual value is defined in variables.cpp
extern const int LED_PIN;

// Motor IO/DSHOT Pins
extern const gpio_num_t global_PIN_MOTOR_FL;
extern const gpio_num_t global_PIN_MOTOR_FR;
extern const gpio_num_t global_PIN_MOTOR_BL;
extern const gpio_num_t global_PIN_MOTOR_BR;

// Battery Monitoring Pins
extern const gpio_num_t global_PIN_BATTERY_VOLTAGE_MONITOR;
extern const gpio_num_t global_PIN_BATTERY_CURRENT_MONITOR;

// IR-Sensor Pins
extern const gpio_num_t global_PIN_IR_SENSOR_FL;
extern const gpio_num_t global_PIN_IR_SENSOR_FR;
extern const gpio_num_t global_PIN_IR_SENSOR_BM;

// I2C Pins (GPIO5 = SDA, GPIO6 = SCL)
extern const gpio_num_t global_PIN_I2C_SDA;
extern const gpio_num_t global_PIN_I2C_SCL;

// *************************************************
// Battery related limits and variables
// *************************************************
// Voltage divider (resistors) ratio: V_battery = V_pin * ratio
extern const float global_BatteryVoltage_VoltageDividerRatio;
extern const float global_BatteryCurrent_VoltageDividerRatio;

// Betaflight-style current scale in units of [0.1 mV] / Amp.
// Example: scale=130 => 13 mV/A => Current(A) = Sensor_mV / 13
extern const float global_BatteryCurrent_SensorScaler_AmpsPerVolt;

// Cutoff/warning voltages
extern const float global_BatteryVoltageLow_WarningLow;            // warning with LED/Wifi when below this voltage
extern const float global_BatteryVoltageLow_MotorCutoffLow;        // motors should stop turning below this voltage
extern const uint16_t global_BatteryVoltageLow_MotorCutoffSamples; // motors are disabled if below cutoff for more than this many samples

// *************************************************
// Motor control variables
// *************************************************
extern const float global_AllMotorsScalePercent; // overall motor power scaler to avoid overloading/burning motors, adjust depending on kV

// Reverse-direction compensation factor.
// If your propellers are less efficient in reverse (negative command), you can boost reverse commands
// by this factor. Applied only when the *requested* motor command is negative.
extern const float global_NegativeRpmScaleFactor;

// *************************************************
// Web piloting / UI presets
// *************************************************
// Lift presets used by the web UI.
// The UI cycles: 0 -> preset[i] -> 0 -> preset[i+1] -> ...
extern const float global_WebLiftPresetPercent_Array[];
extern const size_t global_WebLiftPresetPercent_Array_len;

// Start index for the first non-zero lift preset after startup (startup is always 0).
extern const int global_WebLiftPresetPercent_Array_startIndex;

// Thrust preset: scales the maximum thrust command from the web UI.
// Example: if this is 50, a full-forward (100%) UI command results in 50% thrust command.
extern const float global_WebThrustPresetPercent;

extern bool global_MotorsReversedFL; // Front Left motor reversed
extern bool global_MotorsReversedFR; // Front Right motor normal
extern bool global_MotorsReversedBL; // Back Left motor normal
extern bool global_MotorsReversedBR; // Back Right motor reversed

// *************************************************
// Wifi SSID, PW, IP Adressen
// *************************************************

// Access Point credentials (softAP)
extern const char global_WifiApSsid[];
extern const char global_WifiApPassword[];

// Web UI / webserver port (default: 80)
extern const uint16_t global_WebServerPort;

// *************************************************
// Gyro/IMU/Complementary filter settings
// *************************************************
extern const float global_ComplementaryFilter_yawAlpha; // Complementary filter alpha, high = trust gyro more

// *************************************************
// Control loop constants
// *************************************************
// extern const float f_loop;          // Hz
// extern const float T_loop;          // s

// Main control loop rate for attitude/rate controllers (Hz).
extern const float global_ControlLoopRate_Hz;

// *************************************************
// PID controller constants
// *************************************************

// Yaw RATE PID (setpoint is yaw rate in deg/s).
extern const float global_YawRatePid_Kp;
extern const float global_YawRatePid_Ki;
extern const float global_YawRatePid_Kd;

// Clamp for PID output in mixer units (diffThrust command, -100..100).
extern const float global_YawRatePid_OutputLimit;

// Clamp for integral term (in the same units as output).
extern const float global_YawRatePid_IntegratorLimit;

// Max yaw-rate setpoint in deg/s for full stick/slider deflection (Betaflight-style).
extern const float global_MaxYawRateSetpoint_dps;

// *************************************************
// Dimensions
// *************************************************
// sensor positions
// motor positions

// IR sensor distance a (equilateral triangle side length)
extern const float global_IRSensorDistance_a_meters;

// IR-line sensor thresholds
extern const float global_IRSensor_Threshold;

#endif // HOVERCRAFT_VARIABLES_H
