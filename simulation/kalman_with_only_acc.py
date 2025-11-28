import numpy as np
import matplotlib.pyplot as plt
import random

# ==========================================
# 1. THE KALMAN FILTER
# ==========================================
class RocketKalman:
    def __init__(self):
        self.altitude = 0.0
        self.velocity = 0.0
        self.P = np.array([[100.0, 0.0], [0.0, 100.0]])
        self.Q_altitude = 0.01
        self.Q_velocity = 0.2
        self.R_measure = 3.0

    def predict(self, accel_z, dt):
        old_alt = self.altitude
        self.altitude = old_alt + self.velocity * dt + 0.5 * accel_z * dt**2
        self.velocity = self.velocity + accel_z * dt
        
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = np.array([[self.Q_altitude, 0.0], [0.0, self.Q_velocity]])
        self.P = F @ self.P @ F.T + Q

    def update(self, measured_altitude):
        y = measured_altitude - self.altitude
        H = np.array([[1.0, 0.0]])
        S = H @ self.P @ H.T + self.R_measure
        K = self.P @ H.T @ np.linalg.inv(S)
        
        update_vector = K * y
        self.altitude += update_vector[0, 0]
        self.velocity += update_vector[1, 0]
        
        I = np.eye(2)
        self.P = (I - K @ H) @ self.P

# ==========================================
# 2. PHYSICS CONSTANTS (From OpenRocket)
# ==========================================
MASS_WET = 1.275
MASS_MOTOR_EMPTY = 0.138
MASS_MOTOR_FULL = 0.204
PROPELLANT_MASS = MASS_MOTOR_FULL - MASS_MOTOR_EMPTY
BURN_TIME = 1.94
DIAMETER = 0.078
AREA = np.pi * (DIAMETER/2)**2 
CD = 0.55

def get_thrust(t):
    if t < 0 or t > BURN_TIME: return 0
    if t < 0.1: return 85.9 * (t / 0.1)
    elif t < 1.5: return 82.0 
    else:
        remaining = 1.94 - t
        return 82.0 * (remaining / (1.94 - 1.5))

# ==========================================
# 3. SIMULATION LOOP
# ==========================================
kf = RocketKalman()

dt = 0.01
total_time = 16.0 
current_time = 0.0

# Physics State
true_alt = 0.0
true_vel = 0.0
current_mass = MASS_WET
mass_loss_rate = PROPELLANT_MASS / BURN_TIME

# Accel-Only Integration State (Dead Reckoning)
accel_only_alt = 0.0
accel_only_vel = 0.0

# Sensor Timing
baro_timer = 0.0
BARO_INTERVAL = 0.05

# Data Storage
t_data = []
true_alt_data, true_vel_data, true_accel_data = [], [], []
kalman_alt_data, kalman_vel_data = [], []
accel_only_alt_data, accel_only_vel_data = [], []
meas_accel_data = []
meas_baro_data, meas_baro_t = [], []

print("Running Simulation...")

while current_time <= total_time:
    # --- A. PHYSICS (Truth) ---
    thrust = get_thrust(current_time)
    if current_time < BURN_TIME: current_mass -= mass_loss_rate * dt
    else: current_mass = MASS_WET - PROPELLANT_MASS
        
    drag_force = 0.5 * 1.225 * (true_vel**2) * CD * AREA
    if true_vel < 0: drag_force = -drag_force
    
    gravity_force = current_mass * 9.81
    
    if true_alt <= 0 and thrust < gravity_force and true_vel <= 0:
        true_accel = 0; true_vel = 0; true_alt = 0
    else:
        net_force = thrust - gravity_force - drag_force
        true_accel = net_force / current_mass

    true_vel += true_accel * dt
    true_alt += true_vel * dt
    
    # --- B. SENSORS (Noise) ---
    meas_accel = true_accel + random.gauss(0, 0.6) # MPU6050 Noise
    
    baro_ready = False
    meas_baro = None
    baro_timer += dt
    if baro_timer >= BARO_INTERVAL:
        meas_baro = true_alt + random.gauss(0, 1.0) # MS5611 Noise
        baro_timer = 0.0
        baro_ready = True

    # --- C. ALGORITHMS ---
    
    # 1. Kalman Filter (Fusion)
    kf.predict(meas_accel, dt)
    if baro_ready: kf.update(meas_baro)

    # 2. Accel Only (Dead Reckoning / Integration)
    # We simply integrate the noisy acceleration.
    accel_only_vel += meas_accel * dt
    accel_only_alt += accel_only_vel * dt

    # --- D. STORE DATA ---
    t_data.append(current_time)
    true_alt_data.append(true_alt); true_vel_data.append(true_vel); true_accel_data.append(true_accel)
    kalman_alt_data.append(kf.altitude); kalman_vel_data.append(kf.velocity)
    accel_only_alt_data.append(accel_only_alt); accel_only_vel_data.append(accel_only_vel)
    meas_accel_data.append(meas_accel)
    
    if baro_ready:
        meas_baro_data.append(meas_baro)
        meas_baro_t.append(current_time)
        
    current_time += dt

# ==========================================
# 4. PLOTTING - WINDOW 1 (FUSION)
# ==========================================
print("Plotting Window 1: Sensor Fusion (Kalman)...")

plt.figure(figsize=(12, 10))
plt.suptitle("WINDOW 1: Kalman Filter (Baro + Accel Fusion)", fontsize=16)

# Altitude
plt.subplot(3, 1, 1)
plt.plot(t_data, true_alt_data, 'k--', label='True Altitude')
plt.scatter(meas_baro_t, meas_baro_data, color='orange', s=5, alpha=0.4, label='Barometer')
plt.plot(t_data, kalman_alt_data, 'g-', linewidth=2, label='Kalman Output')
plt.ylabel('Altitude (m)')
plt.legend()
plt.grid(True)

# Velocity
plt.subplot(3, 1, 2)
plt.plot(t_data, true_vel_data, 'k--', label='True Velocity')
plt.plot(t_data, kalman_vel_data, 'b-', linewidth=2, label='Kalman Velocity')
plt.axhline(0, color='red', linestyle=':')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)

# Acceleration
plt.subplot(3, 1, 3)
plt.plot(t_data, meas_accel_data, 'r-', alpha=0.3, label='MPU6050 Input')
plt.plot(t_data, true_accel_data, 'k--', label='True Accel')
plt.ylabel('Accel (m/s^2)')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)

plt.show() # <--- SCRIPT STOPS HERE UNTIL YOU CLOSE THE WINDOW

# ==========================================
# 5. PLOTTING - WINDOW 2 (ACCEL ONLY)
# ==========================================
print("Plotting Window 2: Accelerometer Only (Drift)...")

plt.figure(figsize=(12, 8))
plt.suptitle("WINDOW 2: What if we ONLY use the Accelerometer?", fontsize=16)

# Subplot 1: The Drift (Comparison)
plt.subplot(2, 1, 1)
plt.plot(t_data, true_alt_data, 'k--', label='True Altitude', alpha=0.6)
plt.plot(t_data, accel_only_alt_data, 'm-', linewidth=2, label='Accel Only Integration')
plt.title("The 'Drift' Problem: Integration Error Accumulates over Time")
plt.ylabel('Altitude (m)')
plt.legend()
plt.grid(True)

# Subplot 2: Raw Accelerometer Data (Zoomed In)
plt.subplot(2, 1, 2)
plt.plot(t_data, meas_accel_data, 'r-', alpha=0.5, label='Raw MPU6050 Data')
plt.plot(t_data, true_accel_data, 'k--', label='True Physics')
plt.title("Raw Accelerometer Noise")
plt.ylabel('Acceleration (m/s^2)')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)

plt.show()