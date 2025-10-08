#include <Arduino.h>
#include <unity.h>

// Include all subsystem headers
#include <motor_ctrl.h>
#include <imu.h>
#include <wifi_manager.h>
#include <network_piloting.h>
#include <ir_sensors.h>
#include <battery_monitor.h>
#include <pid_controller.h>
#include <camera_placeholder.h>

// Added definitions for the test
const int LED_PIN = LED_BUILTIN;
const int16_t PEAKSPEED = 750; // select between 0 and +999, extreme values may not work with all ESCs

int numberOfRamps = 4;

// Global subsystem instances
MotorCtrl motorCtrl;

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    // Initialize motorCtrl with front left on GPIO1; use dummy pins for others
    motorCtrl.init(GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4);

    UNITY_BEGIN(); // IMPORTANT LINE!
}

void loop()
{
    // Motor speed ramp test using motor_ctrl functions
    static int16_t motorSpeed = 0;
    static int increaseDirection = 1;

    motorCtrl.setFrontLeftPercent(motorSpeed);
    motorCtrl.setFrontRightPercent(motorSpeed);
    motorCtrl.setBackLeftPercent(motorSpeed);
    motorCtrl.setBackRightPercent(motorSpeed);
    motorCtrl.update();

    motorSpeed += increaseDirection * 5;
    // Reverse direction at peak speed
    if (motorSpeed >= PEAKSPEED || motorSpeed <= -PEAKSPEED)
    {
        increaseDirection *= -1;
        numberOfRamps--;
        if (numberOfRamps <= 0)
        {
            UNITY_END(); // stop unit testing
        }
    }

    digitalWrite(LED_PIN, motorSpeed > 0 ? HIGH : LOW);
    delay(20);
}