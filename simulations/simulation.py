import matplotlib.pyplot as plt
import numpy as np
from util.pid_controller import PositionPID

# define the simulation settings
dt = 0.01
total_time = 25.0
GRAVITY = 9.81
gravity = np.array([0.0, 0.0, -GRAVITY])

# define the rocket's initial state
position = np.zeros(3)
velocity = np.zeros(3)

# create lists to store results for plotting
times = []
positions = []

# PID
target = np.array([5.0, 5.0, 5.0])

max_thrust_acceleration = 20.0

controller = PositionPID(Kp = [1.0, 1.0, 1.0],
                        Ki=[1.0, 0.0, 0.0],
                        Kd = [2.0, 2.0, 2.0],
                        integral_limit = [10.0, 10.0, 10.0]
                        )

# run the simulation one small time step at a time
for step in range(int(total_time / dt)):
    desired_acceleration = controller.update(
        target, position, velocity, dt
    )

    thrust_acceleration = desired_acceleration - gravity

    # find length of vector
    magnitude = np.linalg.norm(thrust_acceleration) 
    if magnitude > max_thrust_acceleration:
        thrust_acceleration *= max_thrust_acceleration / magnitude

    acceleration = thrust_acceleration + gravity

    velocity += acceleration * dt
    position += velocity * dt

    # prevent the rocket from moving below the ground
    if position[2] <= 0.0:
        position[2] = 0.0

        # stop downward motion after reaching the ground
        if velocity[2] < 0.0:
            velocity[2] = 0.0

    # save the current values for plotting
    times.append((step + 1) * dt)
    positions.append(position.copy())


