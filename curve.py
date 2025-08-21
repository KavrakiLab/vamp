import numpy as np
import matplotlib.pyplot as plt

# --- Initial state ---
x0, y0, theta0 = 0.0, 0.0, np.deg2rad(45)  # Start at origin, 45 deg heading

# --- Inputs ---
v = 1.0       # Linear velocity (m/s) - negative for reverse
omega = 2.5    # Angular velocity (rad/s)

# --- Time settings ---
T = 1.0                     # Total time (s)
dt = 0.01                    # Time step (s)
t = np.arange(0, T, dt)      # Time vector

# --- Preallocate arrays ---
x = np.zeros_like(t)
y = np.zeros_like(t)
theta = np.zeros_like(t)

# --- Initial values ---
x[0], y[0], theta[0] = x0, y0, theta0

# --- Simulation ---
if omega != 0:
        R = v / omega
        for i in range(1, len(t)):
                theta[i] = theta0 + omega * t[i]
                x[i] = x0 + R * (np.sin(theta[i]) - np.sin(theta0))
                y[i] = y0 - R * (np.cos(theta[i]) - np.cos(theta0))
else:
        # Straight line case
        for i in range(1, len(t)):
                theta[i] = theta0
                x[i] = x0 + v * t[i] * np.cos(theta0)
                y[i] = y0 + v * t[i] * np.sin(theta0)

# --- Plotting ---
plt.figure(figsize=(8, 6))
plt.plot(x, y, label=f"v={v} m/s, ω={omega} rad/s", linewidth=2)
plt.quiver(x[::200], y[::200], np.cos(theta[::200]), np.sin(theta[::200]),
        color='r', scale=10, label="Heading")
plt.title("Differential Drive Path")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()

