#include "MPU6050SensorInterface.h"

MPU6050SensorInterface::MPU6050SensorInterface(float lpfAlpha)
: alpha(lpfAlpha) {}

void MPU6050SensorInterface::begin() {
    Wire.begin();

    // Wake up MPU6050
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    delay(100);
}

int MPU6050SensorInterface::update() {
    if(readRaw() == 0){
        accelXRaw -= accelXOffset;
        accelYRaw -= accelYOffset;
        accelZRaw -= accelZOffset;

        gyroXRaw -= gyroXOffset;
        gyroYRaw -= gyroYOffset;
        gyroZRaw -= gyroZOffset;

        applyLPF();
        return 0;
    }
    else{
        return -1;
    }
}

int MPU6050SensorInterface::readRaw() {
    Wire.beginTransmission(IMU_ADDR);

    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0) {
        return -1;
    }

    const uint8_t expectedBytes = 14;
    uint8_t received = Wire.requestFrom(IMU_ADDR, expectedBytes, (uint8_t)true);

    if (received != expectedBytes) {
        return -1;
    }

    int16_t ax, ay, az, gx, gy, gz;

    ax = (int16_t)(Wire.read() << 8 | Wire.read());
    ay = (int16_t)(Wire.read() << 8 | Wire.read());
    az = (int16_t)(Wire.read() << 8 | Wire.read());

    Wire.read(); Wire.read(); // discard temperature

    gx = (int16_t)(Wire.read() << 8 | Wire.read());
    gy = (int16_t)(Wire.read() << 8 | Wire.read());
    gz = (int16_t)(Wire.read() << 8 | Wire.read());

    accelXRaw = ax / accelScale;
    accelYRaw = ay / accelScale;
    accelZRaw = az / accelScale;

    gyroXRaw  = gx / gyroScale;
    gyroYRaw  = gy / gyroScale;
    gyroZRaw  = gz / gyroScale;

    return 0;
}


void MPU6050SensorInterface::calibrate() {
    for (int i = 0; i < 300; i++) {
        readRaw();

        accelXOffset += accelXRaw;
        accelYOffset += accelYRaw;
        accelZOffset += accelZRaw;

        gyroXOffset += gyroXRaw;
        gyroYOffset += gyroYRaw;
        gyroZOffset += gyroZRaw;
        
        delay(1);
    }

    accelXOffset /= 300.0f;
    accelYOffset /= 300.0f;
    accelZOffset /= 300.0f;

    gyroXOffset /= 300.0f;
    gyroYOffset /= 300.0f;
    gyroZOffset /= 300.0f;

    accelZOffset -= 1; //Axa aliniata cu gravitatia tebuie sa citeasca 1 G, nu 0.
}

void MPU6050SensorInterface::applyLPF() {
    accelXFiltered = alpha * accelXFiltered + (1 - alpha) * accelXRaw;
    accelYFiltered = alpha * accelYFiltered + (1 - alpha) * accelYRaw;
    accelZFiltered = alpha * accelZFiltered + (1 - alpha) * accelZRaw;

    gyroXFiltered = alpha * gyroXFiltered + (1 - alpha) * gyroXRaw;
    gyroYFiltered = alpha * gyroYFiltered + (1 - alpha) * gyroYRaw;
    gyroZFiltered = alpha * gyroZFiltered + (1 - alpha) * gyroZRaw;
}
