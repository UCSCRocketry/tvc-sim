import math

class PIDController:
    def __init__(self, kP, kI, kD, dt=0.01, integral_limit=7.5):
        self.kP = math.fabs(kP)
        self.kI = math.fabs(kI)
        self.kD = math.fabs(kD)
        self.dt = dt
        self.integral_limit = integral_limit
        self.integral_error = 0.0
        self.previous_error = None


    def reset(self):
        self.integral_error = 0.0
        self.previous_error = None


    def get_output(self, error, rate=None):
        P_output = self.kP * error

        self.integral_error += error * self.dt # integral of error wrt dt
        scaled_limit = self.integral_limit / self.kI if self.kI != 0 else 0.0
        self.integral_error = max(
            min(self.integral_error, scaled_limit), -scaled_limit
        ) # clamp integral error to prevent windup

        I_output = self.kI * self.integral_error

        if rate is not None:
            D_output = self.kD * -rate # see docstring in calculate
        else:
            if self.previous_error is None: self.previous_error = error
            D_output = self.kD * (error - self.previous_error) / self.dt # numerical differentiation

        self.previous_error = error

        return P_output + I_output + D_output
    

    def calculate(self, target, current, rate=None):
        """Calculate the PID output based on the target and current values.
        Arguments:
            target (float): The desired target value.
            current (float): The current value.
            rate (float, optional): The rate of change of the current value. Defaults to None."""
        error = target - current
        return self.get_output(error, rate=rate)
    