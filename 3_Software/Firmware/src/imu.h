// #ifndef IMU_H
// #define IMU_H

// #include <Arduino.h>

// class IMU {
// private:
//     float accelX, accelY, accelZ;
//     float gyroX, gyroY, gyroZ;
//     float roll, pitch, yaw;
    
// public:
//     IMU();
    
//     // Initialization
//     void init();
    
//     // Update function with delta time
//     void update(float dt_s);
    
//     // Accelerometer data
//     void setAccel(float ax, float ay, float az);
//     void getAccel(float* ax, float* ay, float* az);
    
//     // Gyroscope data
//     void setGyro(float gx, float gy, float gz);
//     void getGyro(float* gx, float* gy, float* gz);
    
//     // Angle data (Euler angles)
//     void setAngles(float roll, float pitch, float yaw);
//     void getAngles(float* roll, float* pitch, float* yaw);
    
//     // Individual angle setters
//     void setAngleRoll(float r);
//     void setAnglePitch(float p);
//     void setAngleYaw(float y);
    
//     // Individual angle getters
//     float getRoll();
//     float getPitch();
//     float getYaw();
// };

// #endif // IMU_H