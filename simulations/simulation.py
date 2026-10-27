# libraries
import numpy as np
import matplotlib.pyplot as plt

# define the simulation settings
dt = 0.01
total_time = 10.0
gravity = 9.81

# define the rocket's initial state
height = 0.0
velocity = 0.0

# define a fictional upward acceleration for this classroom model
upward_acceleration = 15.0

# create lists to store results for plotting
times = []
heights = []
velocities = []

# run the simulation one small time step at a time
for step in range(int(total_time / dt)):
    time = step * dt

    # apply the fictional upward acceleration for the first two seconds
    # afterward, only gravity affects the rocket's motion
    if time < 2.0:
        acceleration = upward_acceleration - gravity
    else:
        acceleration = -gravity

    # update velocity based on acceleration and elapsed time
    velocity += acceleration * dt

    # update height based on the new velocity
    height += velocity * dt

    # prevent the rocket from moving below the ground
    if height <= 0:
        height = 0.0

        # stop downward motion after reaching the ground
        if velocity < 0:
            velocity = 0.0

    # save the current values for plotting
    times.append(time)
    heights.append(height)
    velocities.append(velocity)

# create a graph showing the rocket's height over time
plt.plot(times, heights, label="rocket height")
plt.xlabel("time (seconds)")
plt.ylabel("height (meters)")
plt.title("basic 1d rocket motion simulation")
plt.grid(True)
plt.legend()
plt.show()