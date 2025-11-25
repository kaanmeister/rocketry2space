#ifndef MS5611_SENSOR_INTERFACE_H
#define MS5611_SENSOR_INTERFACE_H

#include <Arduino.h>
#include <Wire.h>

class MS5611SensorInterface {
public:
    MS5611SensorInterface(float seaLevelPressure = 1013.25f);

    void init();
    void updateState();

    float getAltitude() const { return alt; }
    float getFilteredAltitude() const { return altFiltered; }

private:
    // ---------- Constants ----------
    static const uint8_t MS5611_ADDR = 0x77;
    static const uint8_t CMD_RESET   = 0x1E;
    static const uint8_t CMD_CONV_D1 = 0x48;
    static const uint8_t CMD_CONV_D2 = 0x50;
    static const uint8_t CMD_ADC_READ = 0x00;
    static const uint8_t CMD_PROM_BASE = 0xA0;

    enum State { WAIT_PRESSURE, WAIT_TEMP, READY };
    State state = READY;

    // ---------- Calibration ----------
    uint16_t C[7] = {0};

    // ---------- Raw and filtered readings ----------
    uint32_t D1 = 0, D2 = 0;
    float D1Filtered = 0;

    int32_t TEMP = 0;
    int32_t P = 0;

    // ---------- Altitude ----------
    float alt = 0;
    float altFiltered = 0;

    // ---------- Settings ----------
    float seaLevelPressure;  // ✔ stored here
    const float alpha = 0.05;

    // ---------- State machine timing ----------
    int pressureReadCount = 0;
    unsigned long convStart = 0;
    const unsigned long convTime = 10;

    // ---------- Internal functions ----------
    uint32_t readADC();
    float computePressureTemp(uint32_t D1, uint32_t D2);
    float pressureToAltitude(float pressure_mbar);

};

#endif
