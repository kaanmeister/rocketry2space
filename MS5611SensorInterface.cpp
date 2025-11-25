#include "MS5611SensorInterface.h"

MS5611SensorInterface::MS5611SensorInterface(float seaLevelPressure)
: seaLevelPressure(seaLevelPressure) // ✔ store input
{
    D1Filtered = 0;
    D1 = D2 = 0;
}

void MS5611SensorInterface::init() {
    Wire.begin();

    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(CMD_RESET);
    Wire.endTransmission();
    delay(3);

    // Read calibration data
    for (uint8_t i = 1; i <= 6; i++) {
        Wire.beginTransmission(MS5611_ADDR);
        Wire.write(CMD_PROM_BASE + i * 2);
        Wire.endTransmission();

        Wire.requestFrom(MS5611_ADDR, 2);
        C[i] = (Wire.read() << 8) | Wire.read();
    }

    // ✔ Get initial temperature so altitude calculations are valid
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(CMD_CONV_D2);
    Wire.endTransmission();
    delay(10);
    D2 = readADC();
}

void MS5611SensorInterface::updateState() {
    unsigned long now = millis();

    switch (state) {
    case READY:
        Wire.beginTransmission(MS5611_ADDR);
        Wire.write(CMD_CONV_D1);
        Wire.endTransmission();

        convStart = now;
        state = WAIT_PRESSURE;
        break;

    case WAIT_PRESSURE:
        if (now - convStart >= convTime) {
            D1 = readADC();
            D1Filtered = alpha * D1 + (1 - alpha) * D1Filtered;
            pressureReadCount++;

            if (pressureReadCount % 8 == 0) {
                Wire.beginTransmission(MS5611_ADDR);
                Wire.write(CMD_CONV_D2);
                Wire.endTransmission();

                convStart = now;
                state = WAIT_TEMP;
            } else {
                alt = computePressureTemp(D1, D2);
                altFiltered = computePressureTemp(D1Filtered, D2);
                state = READY;
            }
        }
        break;

    case WAIT_TEMP:
        if (now - convStart >= convTime) {
            D2 = readADC();
            alt = computePressureTemp(D1, D2);
            altFiltered = computePressureTemp(D1Filtered, D2);
            state = READY;
        }
        break;
    }
}

uint32_t MS5611SensorInterface::readADC() {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(CMD_ADC_READ);
    Wire.endTransmission();

    Wire.requestFrom(MS5611_ADDR, 3);
    return ((uint32_t)Wire.read() << 16) |
           ((uint32_t)Wire.read() << 8) |
           Wire.read();
}

float MS5611SensorInterface::pressureToAltitude(float pressure_mbar) {
    return 44330.0f * (1.0f - pow((pressure_mbar / 100.0f) / seaLevelPressure, 0.1903f));
}

float MS5611SensorInterface::computePressureTemp(uint32_t D1, uint32_t D2) {
    int32_t dT = D2 - ((int32_t)C[5] << 8);
    TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608LL;

    int64_t OFF  = ((int64_t)C[2] << 16) + ((C[4] * (int64_t)dT) >> 7);
    int64_t SENS = ((int64_t)C[1] << 15) + ((C[3] * (int64_t)dT) >> 8);

    P = (((D1 * SENS) >> 21) - OFF) >> 15;

    return pressureToAltitude(P);
}
