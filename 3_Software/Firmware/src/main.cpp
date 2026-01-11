#include <Arduino.h>

// Include file for all global variables
#include "hovercraft_variables.h"

// Include all subsystem headers
#include <motor_ctrl.h>
#include <motor_mixer.h>
// #include <imu.h>
// #include <wifi_manager.h>
// #include <network_piloting.h>
// #include <ir_sensors.h>
// #include <battery_monitor.h>
// #include <pid_controller.h>
// #include <camera_placeholder.h>

// RTOS includes
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

// Motor controller with global power scaler, defined in hovercraft_variables.h!!
MotorCtrl motorCtrl(global_AllMotorsScalePercent); // global power scaler reduces the max power to avoid smoking motors, adjust depending on kV of motors

// Mixer that uses the motor controller
// MotorMixer motorMixer(motorCtrl); // Gives control mixer access to motor controller

// Debug ramp test (matches the standalone sketch behavior)
#define PEAKSPEED 200           // throttle between -999 and 999
static int16_t motorSpeed = 0;
static int increaseDirection = 1;

void setup()
{
  // Serial is handy to confirm the firmware is running
  Serial.begin(115200);
  delay(5000);

  // Init motors
  // Args: fl_pin, fr_pin, bl_pin, br_pin,
  //       reversedFL, reversedFR, reversedBL, reversedBR
  // FL and BR are inverted (true)
  motorCtrl.init(
      global_PIN_MOTOR_FL,
      global_PIN_MOTOR_FR,
      global_PIN_MOTOR_BL,
      global_PIN_MOTOR_BR,
      global_MotorsReversedFL, // flag, if FL motor is reversed or not
      global_MotorsReversedFR, // flag, if FR motor is reversed or not
      global_MotorsReversedBL, // flag, if BL motor is reversed or not
      global_MotorsReversedBR  // flag, if BR motor is reversed or not
  );

  // Init mixer (currently empty, but keeps API consistent)
  //  motorMixer.init();

  // Set mixer inputs
  // motorMixer.setLift(40.0f);       // 40 percent lift
  // motorMixer.setDiffThrust(10.0f); // 10 percent balance
  // motorMixer.setThrust(25.0f);     // 25 percent thrust

  // Test single motors
  // motorCtrl.setAllPercent(25.0f, 0.0f, 0.0f, 0.0f);

  // Start at 0 throttle (arming-safe)
  motorCtrl.tempForDebug_SetAll_directly(0);
}

void loop()
{
  // Ramp motor speed up/down between -PEAKSPEED and +PEAKSPEED
  motorCtrl.tempForDebug_SetAll_directly(motorSpeed);

  motorSpeed += increaseDirection * 5;
  if (motorSpeed >= PEAKSPEED || motorSpeed <= -PEAKSPEED)
  {
    increaseDirection *= -1;
  }

  delay(30);
}
