#ifndef VARIABLES_H
#define VARIABLES_H
// In here, all global variables are defined that can be used to change the main parameters of
// the hovercraft without searching through all the code files.

// *************************************************
// Define all IO-Pins
// *************************************************
// Motor IO/DSHOT Pins
static const gpio_num_t global_PIN_MOTOR_FL = GPIO_NUM_8;
static const gpio_num_t global_PIN_MOTOR_FR = GPIO_NUM_7;
static const gpio_num_t global_PIN_MOTOR_BL = GPIO_NUM_44;
static const gpio_num_t global_PIN_MOTOR_BR = GPIO_NUM_43;

// Battery Monitoring Pins
static const gpio_num_t global_PIN_BATTERY_VOLTAGE_MONITOR = GPIO_NUM_9;
static const gpio_num_t global_PIN_BATTERY_CURRENT_MONITOR = GPIO_NUM_4;

// IR-Sensor Pins
static const gpio_num_t global_PIN_IR_SENSOR_FL = GPIO_NUM_1;
static const gpio_num_t global_PIN_IR_SENSOR_FR = GPIO_NUM_2;
static const gpio_num_t global_PIN_IR_SENSOR_BM = GPIO_NUM_3;

// I2C Pins (GPIO5=SDA, GPIO6=SCL)
static const gpio_num_t global_PIN_I2C_SDA = GPIO_NUM_5;
static const gpio_num_t global_PIN_I2C_SCL = GPIO_NUM_6;

// *************************************************
// Battery limits
// *************************************************
const float global_BatteryVoltageLow_WarningLow = 7.5f;     // (not yet used) warning with LED/Wifi when below this voltage
const float global_BatteryVoltageLow_MotorCutoffLow = 7.0f; // (not yet used) motors should stop turning below this voltage

const float global_BatteryVoltage_DividerRatio = 11.0f;             // (not yet used) voltage divider ratio, depends on the resistors used for voltage devider
const float global_BatteryCurrent_SensorScaler_AmpsPerVolt = 30.0f; // (not yet used) see ESC datasheet

// *************************************************
// Motor control variables
// *************************************************
float global_AllMotorsScalePercent = 40.0; // overall motor power scaler to avoid overloading/burning motors, adjust depending on kV

bool global_MotorsReversedFL = true;  // Front Left motor reversed
bool global_MotorsReversedFR = false; // Front Right motor normal
bool global_MotorsReversedBL = false; // Back Left motor normal
bool global_MotorsReversedBR = true;  // Back Right motor reversed

// *************************************************
// Flags, Mutex
// *************************************************

// *************************************************
// Wifi SSID, PW, IP Adressen
// *************************************************

// *************************************************
// Control loop constants
// *************************************************
// float f_loop = 100;          // Hz
// float T_loop = 1.0 / f_loop; // s

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

#endif // VARIABLES_H