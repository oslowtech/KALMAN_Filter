import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Parameters
dt = 0.1                      # time step
t_total = np.arange(0, 25, dt)
g = 9.81                      # gravity (m/s²)
t_apogee = 17                 # seconds
apogee = 1000                 # meters
t_accel_end = 2               # thrust cutoff
a_thrust = 60                 # thrust acceleration (m/s²)

# Initialize arrays
velocity = np.zeros_like(t_total)
altitude = np.zeros_like(t_total)

# Phase 1: Thrust (0–2s)
idx_accel = t_total <= t_accel_end
velocity[idx_accel] = a_thrust * t_total[idx_accel]

# Phase 2: Coasting (2–17s)
v_burnout = a_thrust * t_accel_end
idx_coast = (t_total > t_accel_end) & (t_total <= t_apogee)
velocity[idx_coast] = v_burnout - g * (t_total[idx_coast] - t_accel_end)

# Phase 3: Descent (after apogee)
v_apogee = velocity[np.where(t_total == t_apogee)[0][0]]
idx_fall = t_total > t_apogee
velocity[idx_fall] = v_apogee - g * (t_total[idx_fall] - t_apogee)

# Integrate velocity to get altitude
for i in range(1, len(t_total)):
    altitude[i] = altitude[i-1] + velocity[i-1] * dt

# Normalize to match apogee at 1200m
scale = apogee / max(altitude)
altitude *= scale
velocity *= scale

# Acceleration from velocity derivative
acceleration = np.gradient(velocity, dt)

# Simulate noisy sensor readings
altitude_noisy = altitude + np.random.normal(0, 50, size=altitude.shape)      # barometer noise
acceleration_noisy = acceleration + np.random.normal(0, 0.8, size=acceleration.shape)  # IMU noise

# Prepare DataFrame
df = pd.DataFrame({
    "time_s": t_total,
    "altitude_true_m": altitude,
    "altitude_noisy_m": altitude_noisy,
    "velocity_mps": velocity,
    "acceleration_true_mps2": acceleration,
    "acceleration_noisy_mps2": acceleration_noisy
})

# Save to CSV
df.to_csv("rocket_dummy_flight.csv", index=False)
print("Dummy rocket flight data saved to 'rocket_dummy_flight.csv'.")

# Optional: Plot to verify
plt.figure(figsize=(10, 5))
plt.plot(t_total, altitude, label="True Altitude")
plt.plot(t_total, altitude_noisy, label="Barometer (Noisy)", linestyle="dotted",linewidth=4)

plt.axvline(t_accel_end, color='gray', linestyle='--', label="Thrust Off")
plt.axvline(t_apogee, color='red', linestyle='--', label="Apogee")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Simulated Rocket Trajectory")
plt.grid(True)
plt.legend()
plt.show()
