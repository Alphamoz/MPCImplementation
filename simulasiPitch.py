import math
import numpy as np
import matplotlib.pyplot as plt

def calc_qdot(q, theta, U_mm):
    W = 687.96
    theta_in_rad = theta
    # print(theta_in_rad)
    return (-100.85 * abs(q) * q - 0.0113 * W * math.sin(theta_in_rad) - (4/70.2) * 7.5e-2 * W * (U_mm-250)/250 * math.cos(theta_in_rad))/28.25

def rad_to_deg(value):
    value = value * 180/math.pi
    return value

q_init = 0
tetha_init = 0
U_mm_init = 500

q = q_init
theta = tetha_init
U_mm = U_mm_init

# Time step dan jumlah iterasi
dt = 1
num_steps = 100

# Simulasi dengan metode Euler
q_values = []
theta_values = []

for i in range(int(num_steps/dt)):
    # Simpan nilai q dan theta
    theta_deg = rad_to_deg(theta)
    q_values.append(q)
    theta_values.append(theta_deg)
    
    # Hitung qdot berdasarkan nilai q dan theta saat ini
    qdot = calc_qdot(q, theta, U_mm)
    
    # Update nilai q dan theta menggunakan metode Euler
    q += qdot * dt
    theta += q*dt
    
# Plot hasil simulasi
time = np.linspace(0, num_steps, num = int(num_steps/dt))
plt.plot(time, theta_values, label='theta')
plt.plot(time, q_values, label='q')
# plt.plot(time, theta_values, label='q')
plt.xlabel('Time')
plt.ylabel('q')
plt.legend()
plt.grid()
plt.show()