#include "hovercraft_variables.h"
// In here, all hovercraft variables/settings are defined that can be used to change the main parameters of
// the hovercraft without searching through all the code files

// *************************************************
// Define all IO-Pins (actual definitions)
// *************************************************
const int LED_PIN = LED_BUILTIN;
const int IR1_PIN = 1; // IR Sensor 1 = Back sensor = GPIO1
const int IR2_PIN = 2; // IR Sensor 2 = Front-Left sensor = GPIO2
const int IR3_PIN = 3; // IR Sensor 3 = Front-Right sensor = GPIO3

// *************************************************
// Battery limits (actual definitions)
// *************************************************
const float global_BatteryVoltageLow_WarningLow = 7.4f;     // warning with LED/Wifi when below this voltage, for 2s LiPo ~3.70V per cell
const float global_BatteryVoltageLow_MotorCutoffLow = 7.0f; // motors should stop turning below this voltage, for 2s LiPo ~3.50V per cell

const float global_BatteryVoltage_DividerRatio = 2.0f;              // voltage divider ratio, depends on the resistors used for voltage divider
const float global_BatteryCurrent_SensorScaler_AmpsPerVolt = 30.0f; // see ESC datasheet

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
