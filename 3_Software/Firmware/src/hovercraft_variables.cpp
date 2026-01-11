#include "hovercraft_variables.h"
// In here, all hovercraft variables/settings are defined that can be used to change the main parameters of
// the hovercraft without searching through all the code files

// *************************************************
// Define all IO-Pins (actual definitions)
// *************************************************
const int LED_PIN = LED_BUILTIN;

// Motor IO/DSHOT Pins
const gpio_num_t global_PIN_MOTOR_FL = GPIO_NUM_8;
const gpio_num_t global_PIN_MOTOR_FR = GPIO_NUM_7;
const gpio_num_t global_PIN_MOTOR_BL = GPIO_NUM_44;
const gpio_num_t global_PIN_MOTOR_BR = GPIO_NUM_43;

// Battery Monitoring Pins
const gpio_num_t global_PIN_BATTERY_VOLTAGE_MONITOR = GPIO_NUM_9;
const gpio_num_t global_PIN_BATTERY_CURRENT_MONITOR = GPIO_NUM_4;

// IR-Sensor Pins
const gpio_num_t global_PIN_IR_SENSOR_FL = GPIO_NUM_1; // Front-Left sensor
const gpio_num_t global_PIN_IR_SENSOR_FR = GPIO_NUM_2; // Front-Right sensor
const gpio_num_t global_PIN_IR_SENSOR_BM = GPIO_NUM_3; // Back sensor

// I2C Pins (GPIO5 = SDA, GPIO6 = SCL)
const gpio_num_t global_PIN_I2C_SDA = GPIO_NUM_5;
const gpio_num_t global_PIN_I2C_SCL = GPIO_NUM_6;

// *************************************************
// Battery related limits and variables(actual definitions)
// *************************************************
// Voltage divider (resistors) ratio: V_battery = V_pin * ratio
const float global_BatteryVoltage_VoltageDividerRatio = 0.3125f; // voltage divider ratio, depends on the resistors used for voltage divider, here 100kOhm/(100kOhm+220kOhm) = 0.3125
const float global_BatteryCurrent_VoltageDividerRatio = 1.0f;    // current sensor voltage divider ratio, if any

// Betaflight-style current scale in units of [0.1 mV] / Amp.
// Example: scale=130 => 13 mV/A => Current(A) = Sensor_mV / 13
const float global_BatteryCurrent_SensorScaler_AmpsPerVolt = 130.0f; // from the ESC datasheet

const float global_BatteryVoltageLow_WarningLow = 7.4f;     // warning with LED/Wifi when below this voltage, for 2s LiPo ~3.70V per cell
const float global_BatteryVoltageLow_MotorCutoffLow = 7.0f; // motors should stop turning below this voltage, for 2s LiPo ~3.50V per cell

// *************************************************
// Motor control variables (actual definitions)
// *************************************************
float global_AllMotorsScalePercent = 40.0f; // overall motor power scaler to avoid overloading/burning motors, adjust depending on kV

bool global_MotorsReversedFL = true;  // Front Left motor reversed
bool global_MotorsReversedFR = false; // Front Right motor normal
bool global_MotorsReversedBL = false; // Back Left motor normal
bool global_MotorsReversedBR = true;  // Back Right motor reversed

// *************************************************
// Wifi SSID, PW, IP Adressen (define here when needed)
// *************************************************

// *************************************************
// Gyro/IMU/Complementary filter settings
// *************************************************
const float global_ComplementaryFilter_yawAlpha = 0.9f; // Complementary filter alpha, high = trust gyro more

// *************************************************
// Control loop constants (example, if you enable them)
// *************************************************
// const float f_loop = 100.0f;          // Hz
// const float T_loop = 1.0f / f_loop;   // s

// *************************************************
// pid;
// *************************************************

// *************************************************
// filter??
// *************************************************

// *************************************************
// Dimensions
// *************************************************
// IR Sensor distance a (actual definition)
const float global_IRSensorDistance_a_meters = 1.0f; // TODO: distance between neighboring IR-sensors (equilateral triangle side length a)

// sensor positions
// motor positions

// *************************************************
// Thresholds, limits, etc.
// *************************************************
// IR-line sensor thresholds
const float global_IRSensor_Threshold = 0.7f; // example threshold value for IR sensors when a line is detected, adjust based on calibration
