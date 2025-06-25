import pandas as pd
import matplotlib.pyplot as plt

# Load results
csv_path = '../../cpp/results/kalman_results.csv'
df = pd.read_csv(csv_path)

time = df['Time']
true_theta = df['True_theta']
true_theta_dot = df['True_theta_dot']
meas_theta = df['Meas_theta']
meas_theta_dot = df['Meas_theta_dot']
est_theta = df['Est_theta']
est_theta_dot = df['Est_theta_dot']

plt.figure(figsize=(12, 5))
plt.plot(time, true_theta, label='True θ', color='blue')
plt.plot(time, meas_theta, label='Measured θ', color='gray', linestyle=':')
plt.plot(time, est_theta, label='Estimated θ', color='orange', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel('θ (Angle)')
plt.title('Position (θ) Estimation with Kalman Filter')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.show()

plt.figure(figsize=(12, 5))
plt.plot(time, true_theta_dot, label=r'True $\dot{\theta}$', color='blue')
plt.plot(time, meas_theta_dot, label=r'Measured $\dot{\theta}$', color='gray', linestyle=':')
plt.plot(time, est_theta_dot, label=r'Estimated $\dot{\theta}$', color='orange', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel('θ̇ (Angular Velocity)')
plt.title('Angular Velocity (θ̇) Estimation with Kalman Filter')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.show()
