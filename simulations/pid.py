import numpy as np

class PositionPID: 
    def __init__(self, Kp, Ki, Kd, integral_limit):
        self.Kp = np.array(Kp, dtype=float)
        self.Ki = np.array(Ki, dtype=float)
        self.Kd = np.array(Kd, dtype=float)
        self.integral_error = np.zeros(3)
        self.integral_limit = np.asarray(integral_limit, dtype=float)

    def update(self, target, position, velocity, dt):
        error = target - position

        self.integral_error += error * dt

        self.integral_error = np.clip(
            self.integral_error,
            -self.integral_limit,
            self.integral_limit,
        )

        return (
            self.Kp * error +
            self.Ki * self.integral_error +
            self.Kd * -velocity
        )
        