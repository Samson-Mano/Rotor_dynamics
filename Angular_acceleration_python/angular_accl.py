import math
import numpy as np
import matplotlib.pyplot as plt


def linear_ramp(total_period, delta_t, max_y, start_time=0, end_time=None):
    if end_time is None:
        end_time = total_period
    num_points = int(total_period / delta_t) + 1
    time_values = np.linspace(start_time, end_time, num_points)
    ramp_array = np.minimum((time_values / total_period) * max_y, max_y)
    return ramp_array

def simulate_disk_motion(r, max_rpm, delta_t, ramp_up_period, ramp_down_period, total_time):
    # Calculate angular velocity
    omega_max = max_rpm * 2.0 * (np.pi / 60.0)
    num_points = int(total_time / delta_t) + 1
    time_values = np.linspace(0, total_time, num_points+1)

    ramp_up_angular_velocity = linear_ramp(ramp_up_period, delta_t, omega_max, 0, ramp_up_period)
    ramp_down_angular_velocity = linear_ramp(ramp_down_period, delta_t, omega_max, total_time - ramp_down_period, total_time)

    angular_velocity = np.concatenate((ramp_up_angular_velocity, ramp_down_angular_velocity))

    # Calculate linear velocity
    linear_velocity = angular_velocity * r

    # Calculate linear acceleration
    d_angular_velocity = np.gradient(angular_velocity, delta_t)
    linear_acceleration = d_angular_velocity * r

    return time_values, angular_velocity, linear_velocity, linear_acceleration

# Inputs
max_rpm = 100
delta_t = 0.01
ramp_up_period = 5
ramp_down_period = 5
total_time = ramp_up_period + ramp_down_period  # Assuming a total time of 20 seconds
eccentricity_r = 5  # Distance from the center

# Simulate disk motion
time_values, angular_velocity, linear_velocity, linear_acceleration = simulate_disk_motion(eccentricity_r, max_rpm, delta_t, ramp_up_period, ramp_down_period, total_time)

# Plotting
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time_values, angular_velocity)
plt.title('Angular Velocity vs Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Angular Velocity (radians per second)')

plt.subplot(3, 1, 2)
plt.plot(time_values, linear_velocity)
plt.title('Linear Velocity vs Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Linear Velocity (units per second)')

plt.subplot(3, 1, 3)
plt.plot(time_values, linear_acceleration)
plt.title('Linear Acceleration vs Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Linear Acceleration (units per second squared)')

plt.tight_layout()
plt.show()