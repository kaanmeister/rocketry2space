#include "MS5611SensorInterface.h"
#include "MPU6050SensorInterface.h"
#include <SoftwareSerial.h>

// Sensors
MS5611SensorInterface ms(1018.7f);
MPU6050SensorInterface imu(0.95f);

// GPS
SoftwareSerial gpsSerial(2, 3);

// TIMING VARIABLES
unsigned long previousSensorMicros = 0;
const unsigned long sensorIntervalMicros = 2000; // 2000 microseconds = 2ms = 500Hz

unsigned long previousGpsMillis = 0;
const unsigned long gpsIntervalMillis = 100000; // 100,000ms = 100 seconds
bool gpsActive = false;
unsigned long gpsWindowStart = 0;

void setup() {
    Serial.begin(115200);
    
    // Initialize GPS but immediately turn it OFF to save CPU for sensors
    gpsSerial.begin(9600);
    gpsSerial.end(); 

    ms.init();
    imu.begin();
    imu.calibrate();

    Serial.println("System Ready. 500Hz Sensor Loop.");
}

void loop() {
    unsigned long currentMicros = micros();
    unsigned long currentMillis = millis();

    // ---------------------------------------------------------
    // 1. HIGH SPEED SENSOR LOOP (500Hz)
    // ---------------------------------------------------------
    if (currentMicros - previousSensorMicros >= sensorIntervalMicros) {
        previousSensorMicros = currentMicros;

        // Update barometer
        ms.updateState();

        // Update IMU
        if (imu.update() != 0){
             // Error handling (minimal print to avoid blocking)
             Serial.println("IMU_ERR");
             imu.begin(); 
        }

        // Get Data
        float altitude = ms.getFilteredAltitude();
        float ax = imu.getAccelX();
        float ay = imu.getAccelY();
        float az = imu.getAccelZ();
        
        // Print Sensor Data (CSV format is best for high speed)
        // Uncommenting this might saturate Serial if too fast
        // Serial.print(altitude); Serial.print(","); Serial.println(az);
    }

    // ---------------------------------------------------------
    // 2. GPS "SNAPSHOT" LOGIC (Every 100 seconds)
    // ---------------------------------------------------------
    
    // Check if it is time to wake up the GPS
    if (!gpsActive && (currentMillis - previousGpsMillis >= gpsIntervalMillis)) {
        previousGpsMillis = currentMillis;
        gpsActive = true;
        gpsWindowStart = currentMillis;
        
        // Turn on GPS interrupts
        gpsSerial.begin(9600); 
        Serial.println("--- GPS WAKEUP ---");
    }

    // If GPS is active, listen for a short window (e.g., 2 seconds) to capture data
    if (gpsActive) {
        // Pass data through if available
        while (gpsSerial.available()) {
            Serial.write(gpsSerial.read());
        }

        // After 2 seconds of listening, shut it down again
        if (currentMillis - gpsWindowStart > 2000) {
            gpsActive = false;
            gpsSerial.end(); // CRITICAL: Stops interrupts so 500Hz loop is clean
            Serial.println("\n--- GPS SLEEP ---");
        }
    }
}