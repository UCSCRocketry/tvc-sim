# libraries
import numpy as np
import matplotlib.pyplot as plt

# define the simulation settings
dt = 0.01
total_time = 10.0
gravity = 9.81

gravity_sim = np.array([0.0, 0.0, -gravity])

# xyz position
xyz_position = np.array([0.0, 0.0, 0.0])

# 3d velocity
velocity = np.array([0.0, 0.0, 0.0])

# define the rocket's initial state
current_height = 0.0
target_position = np.array([5.0, 3.0, 10.0])
mass = 1.0

# define maximum upward acceleration
max_upward_acceleration = 20.0

# create lists to store results for plotting
times = []
positions = []
velocities = []

# PID Gains (ki is now active)
kp = 0.8
ki = 0.4
kd = 0.8

integral_error = np.zeros(3)

# run the simulation one small time step at a time
for step in range(int(total_time / dt)):
    time = step * dt

    # calculate error in xyz
    error = target_position - xyz_position

    # Pid
    p = kp * error
    d = -kd * velocity

    proposed_integral_error = integral_error + (error * dt)
    i = ki * proposed_integral_error

    # target thrust acceleration
    target_acceleration = p + i + d
    upward_acceleration = target_acceleration - gravity_sim
    acceleration_magnitude = np.linalg.norm(upward_acceleration)

    # updating integral accumulator if it hasn't reached max value
    if acceleration_magnitude <= max_upward_acceleration:
        integral_error = proposed_integral_error
    else:
        # no wind up
        pass

    # thrust limits
    if acceleration_magnitude > max_upward_acceleration:
        upward_acceleration = (
            upward_acceleration
            * max_upward_acceleration
            / acceleration_magnitude
        )

    # calculate actual acceleration, including gravity
    acceleration = upward_acceleration + gravity_sim

    # update velocity based on acceleration and elapsed time
    velocity += acceleration * dt

    # update position based on new velocity
    xyz_position += velocity * dt

    # prevent the rocket from moving below the ground
    if xyz_position[2] <= 0:
        xyz_position[2] = 0.0

        # stop downward motion after reaching the ground
        if velocity[2] < 0:
            velocity[2] = 0.0

    # update current height
    current_height = xyz_position[2]

    # save the current values for plotting
    times.append(time)
    positions.append(xyz_position.copy())
    velocities.append(velocity.copy())

# convert positions to numpy array
positions = np.array(positions)

# matplotlib 3d
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# plot 3d 
ax.plot(
    positions[:, 0],
    positions[:, 1],
    positions[:, 2],
    label="rocket trajectory"
)

# starting point
ax.scatter(0, 0, 0, color='green', label="start")

# marking the target point on the graph
ax.scatter(
    target_position[0],
    target_position[1],
    target_position[2],
    color='red',
    marker='x',
    s=100,
    label="target"
)

# graph labels
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.set_zlabel("Height (meters)")

ax.set_title("3D Hopper Simulation")
ax.grid(True)
ax.legend()

plt.show()

# print final results
print("Final position:", xyz_position)
print("Target position:", target_position)
print("Final velocity:", velocity)
