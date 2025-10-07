#include "imu.h"

IMU::IMU() {
    // TODO: Initialize member variables
}

void IMU::init() {
    // TODO: Initialize IMU sensor
}

void IMU::update(float dt_s) {
    // TODO: Update IMU readings
}

void IMU::setAccel(float ax, float ay, float az) {
    // TODO: Set accelerometer values
}

void IMU::getAccel(float* ax, float* ay, float* az) {
    // TODO: Get accelerometer values
}

void IMU::setGyro(float gx, float gy, float gz) {
    // TODO: Set gyroscope values
}

void IMU::getGyro(float* gx, float* gy, float* gz) {
    // TODO: Get gyroscope values
}

void IMU::setAngles(float roll, float pitch, float yaw) {
    // TODO: Set angle values
}

void IMU::getAngles(float* roll, float* pitch, float* yaw) {
    // TODO: Get angle values
}

void IMU::setAngleRoll(float r) {
    // TODO: Set roll angle
}

void IMU::setAnglePitch(float p) {
    // TODO: Set pitch angle
}

void IMU::setAngleYaw(float y) {
    // TODO: Set yaw angle
}

float IMU::getRoll() {
    // TODO: Return roll angle
    return 0.0;
}

float IMU::getPitch() {
    // TODO: Return pitch angle
    return 0.0;
}

float IMU::getYaw() {
    // TODO: Return yaw angle
    return 0.0;
}