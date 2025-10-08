// #ifndef PID_CONTROLLER_H
// #define PID_CONTROLLER_H

// #include <Arduino.h>

// class PIDController {
// private:
//     float kp, ki, kd;        // PID tuning parameters
//     float setpoint;          // Target value
//     float lastInput;         // Last input for derivative calculation
//     float integralSum;       // Integral accumulator
//     float output;            // Controller output
//     float minOutput, maxOutput; // Output limits
//     bool initialized;        // First run flag
    
// public:
//     PIDController();
    
//     // Initialization with PID parameters
//     void init(float kp, float ki, float kd);
    
//     // Reset controller state
//     void reset();
    
//     // PID tuning
//     void setTunings(float kp, float ki, float kd);
//     void getTunings(float* kp, float* ki, float* kd);
    
//     // Setpoint management
//     void setSetpoint(float sp);
//     float getSetpoint();
    
//     // Main computation function
//     float compute(float input, float dt_s);
    
//     // Output management
//     float getOutput();
//     void setOutputLimits(float minOut, float maxOut);
//     void getOutputLimits(float* minOut, float* maxOut);
// };

// #endif // PID_CONTROLLER_H