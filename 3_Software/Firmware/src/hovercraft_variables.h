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

// IR sensor pins (legacy names used by some modules)
extern const int IR1_PIN;
extern const int IR2_PIN;
extern const int IR3_PIN;

// Motor IO/DSHOT Pins
// These are compile time constants, it is fine to keep the definitions in the header
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

// I2C Pins (GPIO5 = SDA, GPIO6 = SCL)
static const gpio_num_t global_PIN_I2C_SDA = GPIO_NUM_5;
static const gpio_num_t global_PIN_I2C_SCL = GPIO_NUM_6;

// *************************************************
// Battery limits
// *************************************************
// Only declared here, defined in variables.cpp
extern const float global_BatteryVoltageLow_WarningLow;     // warning with LED/Wifi when below this voltage
extern const float global_BatteryVoltageLow_MotorCutoffLow; // motors should stop turning below this voltage

extern const float global_BatteryVoltage_DividerRatio;             // voltage divider ratio, depends on the resistors used for voltage divider
extern const float global_BatteryCurrent_SensorScaler_AmpsPerVolt; // see ESC datasheet

// *************************************************
// Motor control variables
// *************************************************
extern float global_AllMotorsScalePercent; // overall motor power scaler to avoid overloading/burning motors, adjust depending on kV

extern bool global_MotorsReversedFL; // Front Left motor reversed
extern bool global_MotorsReversedFR; // Front Right motor normal
extern bool global_MotorsReversedBL; // Back Left motor normal
extern bool global_MotorsReversedBR; // Back Right motor reversed

// *************************************************
// Wifi SSID, PW, IP Adressen
// *************************************************

// *************************************************
// Gyro/IMU/Complementary filter settings
// *************************************************
extern const float global_ComplementaryFilter_yawAlpha; // Complementary filter alpha, high = trust gyro more

// *************************************************
// Control loop constants
// *************************************************
// extern const float f_loop;          // Hz
// extern const float T_loop;          // s

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

// IR sensor distance a (equilateral triangle side length)
extern const float global_IRSensorDistance_a_meters;

// IR-line sensor thresholds
extern const float global_IRSensor_Threshold;

#endif // HOVERCRAFT_VARIABLES_H
