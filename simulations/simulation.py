import matplotlib.pyplot as plt

# define the simulation settings
dt = 0.01
total_time = 25.0
GRAVITY = 9.81

# define the rocket's initial state
y = 0.0
velocity = 0.0

# define a fictional upward acceleration for this classroom model
upward_acceleration = 15.0

# create lists to store results for plotting
times = []
ys = []
velocities = []

# PID
target_y = 3.0
kp = 0.8
ki = 0.0
kd = 0.8

integral_error = 0.0
max_upward_acceleration = 20.0

# run the simulation one small time step at a time
for step in range(int(total_time / dt)):
    time = step * dt

    error = target_y - y

    integral_error += error * dt

    p_term = kp * error
    i_term = ki * integral_error
    d_term = -kd * velocity

    acceleration_needed = GRAVITY + p_term + i_term + d_term

    upward_acceleration = max(0.0, min(max_upward_acceleration, acceleration_needed))
    acceleration = upward_acceleration - GRAVITY

    # update velocity based on acceleration and elapsed time
    velocity += acceleration * dt

    # update y based on the new velocity
    y += velocity * dt

    # prevent the rocket from moving below the ground
    if y <= 0:
        y = 0.0

        # stop downward motion after reaching the ground
        if velocity < 0:
            velocity = 0.0

    # save the current values for plotting
    times.append(time)
    ys.append(y)
    velocities.append(velocity)



# create a graph showing the rocket's y over time
plt.plot(times, ys, label="rocket y")
plt.xlabel("time (seconds)")
plt.ylabel("y (meters)")
plt.title("basic 1d rocket motion simulation")
plt.grid(True)
plt.axhline(target_y, color="red", linestyle="--", label="target height")
plt.legend()
plt.show()
