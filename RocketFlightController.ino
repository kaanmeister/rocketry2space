#include "MS5611SensorInterface.h"
#include "MPU6050SensorInterface.h"
#include <SoftwareSerial.h>

// Sensors
MS5611SensorInterface ms(1018.7f);
MPU6050SensorInterface imu(0.95f);

// GPS
SoftwareSerial gpsSerial(2, 3);

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600); // GPS baud rate

    // Initialize sensors
    ms.init();
    imu.begin();
    imu.calibrate();

    Serial.println("Starting sensor + GPS output (raw NMEA)...");
}

void loop() {
    // Update barometer
    ms.updateState();

    // Update IMU
    if (imu.update() != 0){
        Serial.println("MPU6050 Error, bus disconnected!");
        imu.begin();
    }

    // Read sensor data
    float altitude = ms.getFilteredAltitude();
    float ax = imu.getAccelX();
    float ay = imu.getAccelY();
    float az = imu.getAccelZ();
    float gx = imu.getGyroX();
    float gy = imu.getGyroY();
    float gz = imu.getGyroZ();

    // Print sensor data
    // Serial.print("Alt: "); Serial.print(altitude);
    // Serial.print(" Ax: "); Serial.print(ax);
    // Serial.print(" Ay: "); Serial.print(ay);
    // Serial.print(" Az: "); Serial.print(az);
    // Serial.print(" Gx: "); Serial.print(gx);
    // Serial.print(" Gy: "); Serial.print(gy);
    // Serial.print(" Gz: "); Serial.println(gz);

    // Read and print raw GPS NMEA data
    while (gpsSerial.available()) {
        Serial.write(gpsSerial.read());
    }

    // Small delay to avoid flooding the Serial monitor
    delay(100);
}
