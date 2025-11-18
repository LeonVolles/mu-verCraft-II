#include "hovercraft_variables.h"
// In here, all hovercraft variables/settings are defined that can be used to change the main parameters of
// the hovercraft without searching through all the code files

// *************************************************
// Define all IO-Pins (actual definitions)
// *************************************************
const int LED_PIN = LED_BUILTIN;

// *************************************************
// Battery limits (actual definitions)
// *************************************************
const float global_BatteryVoltageLow_WarningLow = 7.4f;     // warning with LED/Wifi when below this voltage, for 2s LiPo ~3.70V per cell
const float global_BatteryVoltageLow_MotorCutoffLow = 7.0f; // motors should stop turning below this voltage, for 2s LiPo ~3.50V per cell

const float global_BatteryVoltage_DividerRatio = 2.0f;             // voltage divider ratio, depends on the resistors used for voltage divider
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
// sensor positions
// motor positions
