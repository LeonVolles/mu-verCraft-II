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
MotorMixer motorMixer(motorCtrl); // Gives control mixer access to motor controller

// Debug ramp test (matches the standalone sketch behavior)
#define PEAKSPEED_PERCENT 20.0f // percent between -100.0f and 100.0f
static float motorSpeedPercent = 0.0f;
static int increaseDirection = 1;

void setup()
{
  // Serial is handy to confirm the firmware is running
  Serial.begin(115200);
  delay(500);

  // Init motors
  // Args: fl_pin, fr_pin, bl_pin, br_pin,
  //       reversedFL, reversedFR, reversedBL, reversedBR
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

  // Start at 0 throttle (arming-safe)
  delay(20);
  motorCtrl.setAllPercent(0.0f, 0.0f, 0.0f, 0.0f);

  delay(20);

  // Init mixer
  motorMixer.init();

  // Set mixer inputs
  // motorMixer.setLift(40.0f);       // 40 percent lift
  // motorMixer.setDiffThrust(10.0f); // 10 percent balance
  // motorMixer.setThrust(25.0f);     // 25 percent thrust

  // start all motors at low throttle for testing
  motorCtrl.setAllPercent(5.0f, 5.0f, 5.0f, 5.0f);
  delay(100);
}

void loop()
{
  /*   // Test with percent control
    motorCtrl.setAllPercent(motorSpeedPercent, motorSpeedPercent, motorSpeedPercent, motorSpeedPercent);
    motorSpeedPercent += increaseDirection * 1.0f;
    if (motorSpeedPercent >= PEAKSPEED_PERCENT || motorSpeedPercent <= -PEAKSPEED_PERCENT)
    {
      increaseDirection *= -1;
    } */

  /*   // Use motor mixer to test, instead of motorCtrl directly, keep doing ramps tests, for now only in lift
    motorMixer.setLift(motorSpeedPercent);
    motorMixer.setThrust(motorSpeedPercent);
    motorMixer.setDiffThrust(0.0);
    motorSpeedPercent += increaseDirection * 1.0f;
    if (motorSpeedPercent >= PEAKSPEED_PERCENT || motorSpeedPercent <= 0.0f)
    {
      increaseDirection *= -1;
    } */

  motorMixer.setLift(40.0f);
  motorMixer.setThrust(60.0f);

  /*   // Use motor mixer to test, instead of motorCtrl directly, keep doing ramps tests, for now only in lift
    motorMixer.setLift(motorSpeedPercent);
    motorMixer.setThrust(motorSpeedPercent);
    motorMixer.setDiffThrust(0.0);
    if (motorSpeedPercent < PEAKSPEED_PERCENT / 2)
    {
      motorSpeedPercent += increaseDirection * 0.1f;
    }
    else
    {
      motorSpeedPercent = 100.0f;
    }

    if (motorSpeedPercent >= PEAKSPEED_PERCENT || motorSpeedPercent <= 0.0f)
    {
      increaseDirection *= -1;
    } */

  /*   if (motorSpeedPercent < 80.0f)
    {
      motorSpeedPercent += 1.0f;

      motorCtrl.setAllPercent(motorSpeedPercent, motorSpeedPercent, motorSpeedPercent, motorSpeedPercent);


      motorMixer.setLift(motorSpeedPercent);
      motorMixer.setThrust(motorSpeedPercent);
  }
  */

  /*   // motorMixer.setLiftThrustDiff(lift%, thrust%, diffThrust%)
    int mySemiRandom = (millis() / 1000) % 3;
    motorMixer.setLiftThrustDiff(40 + mySemiRandom, 60 + mySemiRandom, 0); */

  delay(30);
}
