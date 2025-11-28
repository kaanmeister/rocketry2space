import random

class RocketKalman:
    def __init__(self):
        # State Vector
        self.altitude = 0.0
        self.velocity = 0.0
        
        # Covariance Matrix (Uncertainty)
        # P = [[100, 0], [0, 100]]
        self.P00 = 100.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 100.0
        
        # Tunable Parameters
        self.Q_altitude = 0.01
        self.Q_velocity = 0.1
        self.R_measure = 3.0

    def predict(self, accel_z, dt):
        # 1. Update State Estimate
        old_alt = self.altitude
        self.altitude = old_alt + self.velocity * dt + 0.5 * accel_z * dt * dt
        self.velocity = self.velocity + accel_z * dt
        
        # 2. Update Covariance
        dt2 = dt * dt
        
        # Backup old values for matrix math
        p00 = self.P00
        p01 = self.P01
        p10 = self.P10
        p11 = self.P11
        
        # P = A*P*A' + Q
        self.P00 = p00 + dt * (p10 + p01) + dt2 * p11 + self.Q_altitude
        self.P01 = p01 + dt * p11
        self.P10 = p10 + dt * p11
        self.P11 = p11 + self.Q_velocity

    def update(self, measured_altitude):
        # 1. Innovation
        y = measured_altitude - self.altitude
        
        # 2. Innovation Covariance (S)
        S = self.P00 + self.R_measure
        
        # 3. Kalman Gain (K)
        K0 = self.P00 / S
        K1 = self.P10 / S
        
        # 4. Update State
        self.altitude = self.altitude + K0 * y
        self.velocity = self.velocity + K1 * y
        
        # 5. Update Covariance
        p00 = self.P00
        p01 = self.P01
        
        self.P00 = p00 - K0 * p00
        self.P01 = p01 - K0 * p01
        self.P10 = self.P10 - K1 * p00
        self.P11 = self.P11 - K1 * p01

# --- RUN SIMULATION ---

kf = RocketKalman()

# Simulation settings
dt = 0.01  # 100Hz
total_time = 10.0
current_time = 0.0

true_alt = 0.0
true_vel = 0.0

# Barometer settings
baro_timer = 0.0
BARO_INTERVAL = 0.05 # 20Hz

print("Time,TrueAlt,NoisyBaro,KalmanAlt,TrueVel,KalmanVel")

while current_time <= total_time:
    # 1. Physics (Truth)
    if current_time < 3.0:
        true_accel = 20.0
    elif true_alt > 0:
        true_accel = -9.81
    else:
        true_accel = 0.0
        true_vel = 0.0
        true_alt = 0.0
        
    true_vel += true_accel * dt
    true_alt += true_vel * dt
    
    # 2. Generate Sensor Data (Add Noise)
    # Accelerometer Noise (+/- 0.5)
    measured_accel = true_accel + random.gauss(0, 0.5)
    
    # Barometer Logic
    baro_ready = False
    measured_baro = 0.0
    
    baro_timer += dt
    if baro_timer >= BARO_INTERVAL:
        # Baro Noise (+/- 1.5)
        measured_baro = true_alt + random.gauss(0, 1.5)
        baro_timer = 0.0
        baro_ready = True
        
    # 3. Run Filter
    kf.predict(measured_accel, dt)
    
    if baro_ready:
        kf.update(measured_baro)
        
    # 4. Print for CSV
    # We only print the "measured baro" if we actually read it, otherwise print the filter value to keep graph clean
    baro_out = measured_baro if baro_ready else kf.altitude 
    
    print(f"{current_time:.2f},{true_alt:.2f},{baro_out:.2f},{kf.altitude:.2f},{true_vel:.2f},{kf.velocity:.2f}")
    
    current_time += dt