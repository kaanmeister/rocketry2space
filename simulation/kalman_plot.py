import numpy as np
import matplotlib.pyplot as plt
import random

# ==========================================
# 1. THE KALMAN FILTER (Unchanged)
# ==========================================
class RocketKalman:
    def __init__(self):
        self.altitude = 0.0
        self.velocity = 0.0
        self.P = np.array([[100.0, 0.0], [0.0, 100.0]])
        # Tuned Q/R for this specific rocket dynamics
        self.Q_altitude = 0.01
        self.Q_velocity = 0.2  # Increased slightly for high drag deceleration
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
# 2. OPENROCKET PHYSICS MODEL
# ==========================================

# --- ROCKET PARAMETERS (From your Screenshots) ---
MASS_WET = 1.275        # kg (Start mass)
MASS_MOTOR_EMPTY = 0.138 # kg
MASS_MOTOR_FULL = 0.204  # kg
PROPELLANT_MASS = MASS_MOTOR_FULL - MASS_MOTOR_EMPTY # 0.066 kg
BURN_TIME = 1.94        # Seconds
DIAMETER = 0.078        # Meters (78mm)
AREA = np.pi * (DIAMETER/2)**2 
CD = 0.55               # Estimated Drag Coefficient to match 308m apogee

# --- THRUST CURVE FUNCTION ---
# Based on the Pro38-1G thrust curve image you provided
def get_thrust(t):
    if t < 0: return 0
    if t > BURN_TIME: return 0
    
    # 0.0 to 0.1s: Spike to Max Thrust (85.9 N)
    if t < 0.1: 
        return 85.9 * (t / 0.1)
    # 0.1 to 1.5s: Roughly constant/wobbly around 80 N
    elif t < 1.5:
        return 82.0 
    # 1.5 to 1.94s: Rapid drop off
    else:
        # Linear drop from 82 to 0
        remaining = 1.94 - t
        return 82.0 * (remaining / (1.94 - 1.5))

# ==========================================
# 3. SIMULATION LOOP
# ==========================================
kf = RocketKalman()

dt = 0.01  # 100Hz
total_time = 16.0 
current_time = 0.0

# Initial Physics State
true_alt = 0.0
true_vel = 0.0
current_mass = MASS_WET
mass_loss_rate = PROPELLANT_MASS / BURN_TIME

# Sensor Timing
baro_timer = 0.0
BARO_INTERVAL = 0.05 # 20Hz

# Data Storage
t_data, true_alt_data, kalman_alt_data = [], [], []
true_vel_data, kalman_vel_data = [], []
meas_accel_data, true_accel_data = [], []
meas_baro_data, meas_baro_t = [], []

print(f"Simulating Flight: {MASS_WET}kg Rocket, {BURN_TIME}s burn...")

while current_time <= total_time:
    
    # --- A. CALCULATE PHYSICS (True Flight) ---
    
    # 1. Thrust
    thrust = get_thrust(current_time)
    
    # 2. Mass (Decreases during burn)
    if current_time < BURN_TIME:
        current_mass -= mass_loss_rate * dt
    else:
        current_mass = MASS_WET - PROPELLANT_MASS
        
    # 3. Drag Force (Opposes Velocity) F = 0.5 * rho * v^2 * Cd * A
    rho = 1.225 # Air density
    # Direction is opposite to velocity
    drag_force = 0.5 * rho * (true_vel**2) * CD * AREA
    if true_vel < 0: drag_force = -drag_force # Drag points up if falling down
    
    # 4. Net Force & Acceleration
    gravity_force = current_mass * 9.81
    
    if true_alt <= 0 and thrust < gravity_force and true_vel <= 0:
        # Rocket hasn't lifted off yet or has landed
        true_accel = 0
        true_vel = 0
        true_alt = 0
    else:
        # Newton's 2nd Law: F = ma  ->  a = F/m
        net_force = thrust - gravity_force - drag_force
        true_accel = net_force / current_mass

    # 5. Euler Integration
    true_vel += true_accel * dt
    true_alt += true_vel * dt
    
    # --- B. SENSORS (Noise) ---
    # IMU Noise (+/- 0.6 m/s^2 typical for MPU6050)
    meas_accel = true_accel + random.gauss(0, 0.6)
    
    baro_ready = False
    meas_baro = None
    baro_timer += dt
    
    if baro_timer >= BARO_INTERVAL:
        # Baro Noise (+/- 1.0m typical for MS5611)
        meas_baro = true_alt + random.gauss(0, 1.0) 
        baro_timer = 0.0
        baro_ready = True

    # --- C. KALMAN FILTER ---
    kf.predict(meas_accel, dt)
    if baro_ready:
        kf.update(meas_baro)

    # --- D. STORE DATA ---
    t_data.append(current_time)
    true_alt_data.append(true_alt)
    kalman_alt_data.append(kf.altitude)
    true_vel_data.append(true_vel)
    kalman_vel_data.append(kf.velocity)
    meas_accel_data.append(meas_accel)
    true_accel_data.append(true_accel)
    
    if baro_ready:
        meas_baro_data.append(meas_baro)
        meas_baro_t.append(current_time)
        
    current_time += dt

# ==========================================
# 4. PLOTTING
# ==========================================
print(f"Simulation Complete.")
print(f"Max True Altitude: {max(true_alt_data):.2f} m")
print(f"Max True Velocity: {max(true_vel_data):.2f} m/s")
print(f"Max True Accel:    {max(true_accel_data):.2f} m/s^2")

plt.figure(figsize=(12, 10))

# Altitude
plt.subplot(3, 1, 1)
plt.plot(t_data, true_alt_data, 'k--', label='OpenRocket Physics (Truth)')
plt.scatter(meas_baro_t, meas_baro_data, color='orange', s=5, alpha=0.4, label='MS5611 Data')
plt.plot(t_data, kalman_alt_data, 'g-', linewidth=2, label='Kalman Output')
plt.ylabel('Altitude (m)')
plt.title(f'Altitude (Apogee: {max(kalman_alt_data):.1f}m)')
plt.legend()
plt.grid(True)

# Velocity
plt.subplot(3, 1, 2)
plt.plot(t_data, true_vel_data, 'k--', label='True Velocity')
plt.plot(t_data, kalman_vel_data, 'b-', linewidth=2, label='Kalman Velocity')
plt.axhline(0, color='red', linestyle=':')
plt.ylabel('Velocity (m/s)')
plt.title(f'Velocity (Max: {max(kalman_vel_data):.1f} m/s)')
plt.legend()
plt.grid(True)

# Acceleration
plt.subplot(3, 1, 3)
plt.plot(t_data, meas_accel_data, 'r-', alpha=0.3, label='MPU6050 (Noisy)')
plt.plot(t_data, true_accel_data, 'k--', label='True Accel')
plt.ylabel('Accel (m/s^2)')
plt.xlabel('Time (s)')
plt.title(f'Acceleration (Max: {max(true_accel_data):.1f} m/s^2)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()