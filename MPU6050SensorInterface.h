#ifndef MPU6050_SENSOR_INTERFACE_H
#define MPU6050_SENSOR_INTERFACE_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050SensorInterface {
public:
    MPU6050SensorInterface(float lpfAlpha = 0.95f);
    void calibrate();

    void begin();
    int update();

    // Accelerometer getters (g)
    float getAccelX() const { return accelXFiltered; }
    float getAccelY() const { return accelYFiltered; }
    float getAccelZ() const { return accelZFiltered; }

    // Gyroscope getters (deg/sec)
    float getGyroX() const { return gyroXFiltered; }
    float getGyroY() const { return gyroYFiltered; }
    float getGyroZ() const { return gyroZFiltered; }

private:
    const uint8_t IMU_ADDR = 0x68;

    const float accelScale = 16384.0f; // ±2g
    const float gyroScale  = 131.0f;   // ±250 dps

    float alpha; // LPF α

    // Raw readings
    float accelXRaw = 0, accelYRaw = 0, accelZRaw = 0;
    float gyroXRaw  = 0, gyroYRaw  = 0, gyroZRaw  = 0;

    // Offsets (from calibration)
    float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;
    float gyroXOffset  = 0, gyroYOffset  = 0, gyroZOffset  = 0;

    // Filtered values
    float accelXFiltered = 0, accelYFiltered = 0, accelZFiltered = 0;
    float gyroXFiltered  = 0, gyroYFiltered  = 0, gyroZFiltered  = 0;

    int readRaw();
    void applyLPF();
};

#endif
