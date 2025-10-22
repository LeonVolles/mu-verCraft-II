#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

class IMU
{
private:
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;

public:
    // Constructor
    IMU();

    // Initialization
    void init();

    // Reset global IMU coordinate system
    void resetGlobalCoordSystem();

    // Update function with delta time
    void update(float dt_s);

    // ***Gyroscope data***
    void getGyro(float *gx, float *gy, float *gz);

    // Angle data (Euler angles)
    void setAngles(float roll, float pitch, float yaw);
    void getAngles(float *roll, float *pitch, float *yaw);

    // Individual angle getters
    float getAngleRoll();
    float getAnglePitch();
    float getAngleYaw();

    // ***Accelerometer data***
    // Accelerations x,y,z
    void getAccel(float *ax, float *ay, float *az);

    // Position data x,y,z
    void getPosition(float *x, float *y, float *z);
    void setPosition(float x, float y, float z);

    // Individual position setters
    void setPositionX(float x);
    void setPositionY(float y);
    void setPositionZ(float z);
};

#endif // IMU_H