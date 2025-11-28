#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip> // For formatting output

// ==========================================
// ROCKET KALMAN FILTER CLASS 
// (Standard C++ version - no Arduino libs)
// ==========================================
class RocketKalman {
public:
    // State Vector
    float altitude = 0.0f; 
    float velocity = 0.0f; 

    // Covariance Matrix
    float P[2][2] = { {100.0f, 0.0f}, {0.0f, 100.0f} };

    // Tunable Parameters
    float Q_altitude = 0.01f; 
    float Q_velocity = 0.1f;  
    float R_measure  = 3.0f;  

    void begin(float initialAltitude) {
        altitude = initialAltitude;
        velocity = 0.0f;
    }

    void predict(float accelZ, float dt) {
        // 1. Update State Estimate
        float oldAlt = altitude;
        altitude = oldAlt + velocity * dt + 0.5f * accelZ * dt * dt;
        velocity = velocity + accelZ * dt;

        // 2. Update Covariance
        float dt2 = dt * dt;
        float p00 = P[0][0]; float p01 = P[0][1]; float p10 = P[1][0]; float p11 = P[1][1];

        P[0][0] = p00 + dt * (p10 + p01) + dt2 * p11 + Q_altitude;
        P[0][1] = p01 + dt * p11;
        P[1][0] = p10 + dt * p11;
        P[1][1] = p11 + Q_velocity;
    }

    void update(float measuredAltitude) {
        // 1. Innovation
        float y = measuredAltitude - altitude;

        // 2. Innovation Covariance
        float S = P[0][0] + R_measure;

        // 3. Kalman Gain
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // 4. Update State
        altitude = altitude + K[0] * y;
        velocity = velocity + K[1] * y;

        // 5. Update Covariance
        float p00 = P[0][0]; float p01 = P[0][1];
        P[0][0] = p00 - K[0] * p00;
        P[0][1] = p01 - K[0] * p01;
        P[1][0] = P[1][0] - K[1] * p00;
        P[1][1] = P[1][1] - K[1] * p01;
    }
};

// ==========================================
// MAIN SIMULATION LOOP
// ==========================================
int main() {
    RocketKalman kf;
    kf.begin(0.0f);

    // Setup Random Number Generation (Standard C++)
    std::default_random_engine generator;
    std::normal_distribution<float> accelNoise(0.0, 0.5); // Mean 0, SD 0.5
    std::normal_distribution<float> baroNoise(0.0, 1.5);  // Mean 0, SD 1.5

    // Simulation Constants
    float dt = 0.01f; // 10ms loop time (100Hz)
    float totalTime = 10.0f; // Simulate 10 seconds of flight
    float currentTime = 0.0f;

    // Truth Variables
    float trueAlt = 0.0f;
    float trueVel = 0.0f;
    float trueAccel = 0.0f;

    // Barometer Timing
    float baroTimer = 0.0f;
    const float BARO_INTERVAL = 0.05f; // 20Hz update rate

    // Print CSV Header
    std::cout << "Time,TrueAlt,NoisyBaro,KalmanAlt,TrueVel,KalmanVel" << std::endl;

    // Run Simulation Step-by-Step
    while (currentTime <= totalTime) {
        
        // --- 1. GENERATE PHYSICS (TRUTH) ---
        if (currentTime < 3.0f) {
            trueAccel = 20.0f; // Motor Burn
        } else if (trueAlt > 0.0f) {
            trueAccel = -9.81f; // Coasting/Falling
        } else {
            trueAccel = 0.0f;
            trueVel = 0.0f;
            trueAlt = 0.0f; // Landed
        }

        trueVel += trueAccel * dt;
        trueAlt += trueVel * dt;

        // --- 2. GENERATE SENSOR DATA ---
        float measuredAccel = trueAccel + accelNoise(generator);
        
        // Barometer logic (runs slower than loop)
        bool baroReady = false;
        float measuredBaro = 0.0f;
        
        baroTimer += dt;
        if (baroTimer >= BARO_INTERVAL) {
            measuredBaro = trueAlt + baroNoise(generator);
            baroTimer = 0.0f;
            baroReady = true;
        }

        // --- 3. RUN KALMAN FILTER ---
        kf.predict(measuredAccel, dt);

        if (baroReady) {
            kf.update(measuredBaro);
        }

        // --- 4. OUTPUT DATA (CSV Format) ---
        std::cout << std::fixed << std::setprecision(4) 
                  << currentTime << ","
                  << trueAlt << ","
                  << (baroReady ? measuredBaro : kf.altitude) << "," // Hold value if no new data for cleaner graph
                  << kf.altitude << ","
                  << trueVel << ","
                  << kf.velocity << std::endl;

        currentTime += dt;
    }

    return 0;
}